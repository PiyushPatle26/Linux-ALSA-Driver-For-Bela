// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bela Gem Multi Audio Cape — ASoC machine driver for AM625
 *
 * Three DAI links on a shared McASP2 TDM bus (8 slots x 32 bits, DSP_A = 1-bit data delay).
 * The AIC3106 is the BCLK + WCLK (frame) master; its PLL generates BCLK/WCLK and
 * AUX_MCLK (GPIO1) so the ES9080Q sees an integer MCLK:BCLK ratio. The McASP2 and
 * the ES9080Q/ADC3140s are clock consumers.
 *
 *   Link 0: McASP2 <-> { AIC3106 (stereo P+C), ES9080Q (8-ch playback) }
 *           AIC3106 AXR0 TX / AXR1 RX (slots 0-1); ES9080Q AXR6 TX (slots 0-7).
 *           Both codecs share one stream so the AIC3106 PLL (hence AUX_MCLK) is
 *           running before the ES9080Q register init runs in the same path.
 *   Link 1: McASP2  <- TLV320ADC3140 #1 (4-ch capture)  AXR4 RX (slots 0-3)
 *   Link 2: McASP2  <- TLV320ADC3140 #2 (4-ch capture)  AXR8 RX (slots 0-3)
 *
 * Playback channel map: davinci-mcasp interleaves a multi-serializer stream
 * round-robin, channel n -> serializer (n % active), slot (n / active). With
 * both TX serializers active (16-ch stream = 2 serializers x 8 slots):
 *
 *   even channels -> AXR0 (AIC3106):  ch0 = Line/HP L, ch2 = Line/HP R,
 *                                     ch4..ch14 land in slots 2-7 (unused)
 *   odd  channels -> AXR6 (ES9080Q):  ch1 = OUT1, ch3 = OUT2, ... ch15 = OUT8
 *
 * A 1/2-channel stream activates only AXR0 and plays through the AIC3106
 * alone. Counts of 3-8 would also fit one serializer -- all on AXR0, never
 * reaching the DAC -- so the startup hook constrains playback to {1, 2, 16}.
 * This mirrors Bela's shipping configuration (outSerializers axr0 mask 0x3 +
 * axr6 mask 0xff, 16 output channels).
 *
 * Authors: Piyush Patle <piyushpatle1228@gmail.com>
 *          Giulio Moro <giuliomoro@gmail.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

/*
 * 8 slots x 32 bits = 256 BCLK/frame (= 256 * fs). This matches Bela's
 * Es9080_Codec (kNumBits = 256, kSlotSize = 32 -> kNumSlots = 8) and is the
 * frame the McASP generates as bit-clock/frame master (12.288 MHz BCLK @ 48
 * kHz). The widest codec (ES9080Q, 8 ch) fills all 8 slots; the AIC3106 (2 ch)
 * and each ADC3140 (4 ch) use the low slots of their own serializer.
 */
#define BELA_TDM_SLOTS		8
#define BELA_TDM_SLOT_WIDTH	32

/* Per-codec slot masks (each codec sits on its own serializer, from slot 0) */
#define BELA_AIC_MASK		0x0003U	/* 2 ch  (slots 0-1) */
#define BELA_DAC_MASK		0x00FFU	/* 8 ch  (slots 0-7) */
#define BELA_ADC_MASK		0x000FU	/* 4 ch  (slots 0-3) */

#define BELA_MCLK_DEFAULT	12288000U

/*
 * TLV320AIC3106 GPIO1 control register (page 0, register 98). Writing
 * BELA_AIC3X_GPIO1_PLL_CLKOUT routes the codec's PLL clock to the GPIO1 pad.
 * On Bela Gem Multi that pad (PLL_AUXOUT) is the ONLY MCLK source for the
 * ES9080Q and the two ADC3140s -- their register ports stay inaccessible until
 * it runs. Value matches Bela's userspace driver (I2c_Codec.cpp: reg 98 = 0x50,
 * "GPIO1: clock mux output (PLL) divided"). The AIC3106 PLL must be active for
 * this to produce a clock, which is why the AIC3106 has to keep clocking.
 */
#define BELA_AIC3X_GPIO1_REG		98
#define BELA_AIC3X_GPIO1_PLL_CLKOUT	0x50

/*
 * TLV320AIC3106 audio-serial control register B (page 0, register 9), bit 3:
 * "256-clock mode". When the AIC3106 is the BCLK master this makes it emit a
 * 256-BCLK frame (8 slots x 32 bit) instead of its native ~32/64-clock stereo
 * frame. Matches Bela's I2c_Codec.cpp. The AIC3106 must master BCLK/WCLK so
 * that they and AUX_MCLK all derive from the same AIC3106 PLL -- the ES9080Q
 * requires an integer MCLK:BCLK ratio, which only holds when both come from one
 * clock source.
 */
#define BELA_AIC3X_INTF_CTRLB_REG	9
#define BELA_AIC3X_256_CLOCK_MODE	0x08

/*
 * Link 0 carries TWO codecs (AIC3106 + ES9080Q) so a single playback stream
 * brings both up together. This is required for the ES9080Q: its register port
 * only responds while a master clock (AUX_MCLK) is present, and AUX_MCLK is the
 * AIC3106's PLL clock output (GPIO1) which only runs while the AIC3106 is
 * streaming. The McASP is a single instance, so the AIC3106 and ES9080Q cannot
 * stream on separate links at the same time -- putting them on one link makes
 * opening it spin up the AIC3106 PLL (hence AUX_MCLK) before the ES9080Q init
 * runs in the same hw_params().
 */
struct bela_gem_priv {
	struct snd_soc_card		 card;
	struct snd_soc_dai_link		 dai_links[3];
	struct snd_soc_dai_link_component cpu[3];
	struct snd_soc_dai_link_component platform[3];
	struct snd_soc_dai_link_component play_codecs[2];	/* AIC3106, ES9080Q */
	struct snd_soc_dai_link_component adc1_codec;
	struct snd_soc_dai_link_component adc2_codec;
	struct snd_soc_codec_conf	 codec_conf[2];
	struct clk			*mclk;
	unsigned int			 mclk_freq;
};

static const struct snd_soc_dapm_widget bela_gem_widgets[] = {
	SND_SOC_DAPM_HP("HP Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_OUTPUT("DAC Out 1"),
	SND_SOC_DAPM_OUTPUT("DAC Out 2"),
	SND_SOC_DAPM_OUTPUT("DAC Out 3"),
	SND_SOC_DAPM_OUTPUT("DAC Out 4"),
	SND_SOC_DAPM_OUTPUT("DAC Out 5"),
	SND_SOC_DAPM_OUTPUT("DAC Out 6"),
	SND_SOC_DAPM_OUTPUT("DAC Out 7"),
	SND_SOC_DAPM_OUTPUT("DAC Out 8"),
};

static const struct snd_soc_dapm_route bela_gem_routes[] = {
	{ "HP Jack",   NULL, "HPLOUT" },
	{ "HP Jack",   NULL, "HPROUT" },
	{ "Line Out",  NULL, "LLOUT"  },
	{ "Line Out",  NULL, "RLOUT"  },
	{ "LINE1L",    NULL, "Line In" },
	{ "LINE1R",    NULL, "Line In" },
	{ "DAC Out 1", NULL, "OUT1" },
	{ "DAC Out 2", NULL, "OUT2" },
	{ "DAC Out 3", NULL, "OUT3" },
	{ "DAC Out 4", NULL, "OUT4" },
	{ "DAC Out 5", NULL, "OUT5" },
	{ "DAC Out 6", NULL, "OUT6" },
	{ "DAC Out 7", NULL, "OUT7" },
	{ "DAC Out 8", NULL, "OUT8" },
	/*
	 * ADC3140 capture routing (MIC1P..MIC4M -> capture) is internal to the
	 * tlv320adcx140 codec and configured via its DT input properties, so no
	 * machine-level routes are needed here.
	 */
};

static const struct snd_kcontrol_new bela_gem_controls[] = {
	SOC_DAPM_PIN_SWITCH("HP Jack"),
	SOC_DAPM_PIN_SWITCH("Line Out"),
	SOC_DAPM_PIN_SWITCH("Line In"),
	SOC_DAPM_PIN_SWITCH("DAC Out 1"),
	SOC_DAPM_PIN_SWITCH("DAC Out 2"),
	SOC_DAPM_PIN_SWITCH("DAC Out 3"),
	SOC_DAPM_PIN_SWITCH("DAC Out 4"),
	SOC_DAPM_PIN_SWITCH("DAC Out 5"),
	SOC_DAPM_PIN_SWITCH("DAC Out 6"),
	SOC_DAPM_PIN_SWITCH("DAC Out 7"),
	SOC_DAPM_PIN_SWITCH("DAC Out 8"),
};

/*
 * Link 0 init: McASP2 + { AIC3106, ES9080Q }. The AIC3106 is the BCLK/WCLK
 * master: its PLL generates BCLK/WCLK *and* AUX_MCLK (on GPIO1), so the ES9080Q
 * sees an integer MCLK:BCLK ratio from one source. The McASP is a clock
 * consumer; the ES9080Q is a pure TDM/clock consumer that inits in hw_params.
 */
static int bela_gem_play_init(struct snd_soc_pcm_runtime *rtd)
{
	struct bela_gem_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *aic = snd_soc_rtd_to_codec(rtd, 0);	/* codec 0 */
	struct snd_soc_dai *dac = snd_soc_rtd_to_codec(rtd, 1);	/* codec 1 */
	int ret;

	ret = snd_soc_dai_set_sysclk(aic, 0, priv->mclk_freq, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	/*
	 * Codec-side TDM masks are per DAI and stable, so they are set once
	 * here. The McASP's masks are NOT set here: davinci-mcasp keeps one
	 * global tx/rx mask pair for all three links, so the CPU masks are
	 * programmed from each link's hw_params instead (which runs before the
	 * CPU DAI's own hw_params), keeping whichever stream is being
	 * configured authoritative.
	 */
	ret = snd_soc_dai_set_tdm_slot(aic, BELA_AIC_MASK, BELA_AIC_MASK,
				       BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
	if (ret)
		return ret;

	/*
	 * The stored tx_mask also makes ASoC fix up the ES9080Q's hw_params
	 * to its 8 slots instead of the full interleaved stream width.
	 */
	return snd_soc_dai_set_tdm_slot(dac, BELA_DAC_MASK, 0,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

/*
 * Valid playback channel counts. davinci-mcasp activates TX serializers in
 * index order, DIV_ROUND_UP(channels, slots) at a time: 1-2 channels use AXR0
 * alone (AIC3106 stereo), 16 use AXR0 + AXR6 and reach all ten outputs.
 * Intermediate counts (3-8) would also fit a single serializer, putting every
 * channel on AXR0 where only slots 0-1 are consumed -- silently discarding
 * most of the stream -- so they are excluded.
 */
static const unsigned int bela_gem_play_channels[] = { 1, 2, 16 };

static const struct snd_pcm_hw_constraint_list bela_gem_play_constraints = {
	.list	= bela_gem_play_channels,
	.count	= ARRAY_SIZE(bela_gem_play_channels),
};

static int bela_gem_play_startup(struct snd_pcm_substream *substream)
{
	/*
	 * Capture on this link is the AIC3106's two channels on AXR1 only.
	 * Without a constraint the multi-codec CPU-range bypass in
	 * snd_soc_runtime_calc_hw() would advertise up to 24 channels (3 RX
	 * serializers x 8 slots), and a >2-channel capture would silently
	 * activate the ADC3140s' serializers and interleave their data in.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return snd_pcm_hw_constraint_minmax(substream->runtime,
						    SNDRV_PCM_HW_PARAM_CHANNELS,
						    1, 2);

	return snd_pcm_hw_constraint_list(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_CHANNELS,
					  &bela_gem_play_constraints);
}

/*
 * davinci-mcasp stores a single global TDM mask pair, shared by all three
 * links. Program the McASP masks from link hw_params — it runs before the CPU
 * DAI's own hw_params, so the masks in force always belong to the stream
 * being configured, whichever link last touched them.
 */
static int bela_gem_play_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);

	/*
	 * TX spans the full 8-slot frame on every active serializer; RX is
	 * the AIC3106 stereo capture (slots 0-1 on AXR1).
	 */
	return snd_soc_dai_set_tdm_slot(cpu, BELA_DAC_MASK, BELA_AIC_MASK,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

/*
 * Re-route the AIC3106 PLL clock to its GPIO1 pad (= AUX_MCLK for the ES9080Q
 * and ADC3140s) after hw_params has finished. The aic3x "GPIO1 dmic modclk"
 * DAPM widget also owns GPIO1[7:4], and snd_soc_dapm_update_dai() can restore
 * those bits to "disabled" after a link hw_params write. Link prepare runs
 * after all codec hw_params/DAPM updates and before the ES9080Q DAI prepare,
 * so the clock output is stable before the ES9080Q register port is accessed.
 */
static int bela_gem_play_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *aic = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	/*
	 * Put the AIC3106 in 256-clock mode so, as BCLK master, it emits the
	 * 256-BCLK/8-slot frame the ES9080Q and ADC3140s expect. aic3x clears
	 * this field in hw_params, so set it here (prepare runs afterwards).
	 */
	ret = snd_soc_component_update_bits(aic->component,
					    BELA_AIC3X_INTF_CTRLB_REG,
					    BELA_AIC3X_256_CLOCK_MODE,
					    BELA_AIC3X_256_CLOCK_MODE);
	if (ret < 0)
		return ret;

	return snd_soc_component_write(aic->component, BELA_AIC3X_GPIO1_REG,
				      BELA_AIC3X_GPIO1_PLL_CLKOUT);
}

static const struct snd_soc_ops bela_gem_play_ops = {
	.startup   = bela_gem_play_startup,
	.hw_params = bela_gem_play_hw_params,
	.prepare   = bela_gem_play_prepare,
};

/* Link 1/2 init: McASP2 + TLV320ADC3140 (capture, clock consumer) */
static int bela_gem_adc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *adc = snd_soc_rtd_to_codec(rtd, 0);

	/*
	 * The ADC3140 is capture-only: on the TDM bus the codec TRANSMITS its
	 * captured channels, so the slot mask must be the codec's tx_mask (not
	 * rx_mask). The adcx140 driver validates tx_mask and requires it to be
	 * adjacent slots from slot 0 (GENMASK) — satisfied here since each ADC
	 * sits on its own serializer using slots 0-3. Passing it as rx_mask
	 * leaves tx_mask=0, which the driver rejects with -EINVAL.
	 */
	return snd_soc_dai_set_tdm_slot(adc, BELA_ADC_MASK, 0,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

static int bela_gem_adc_startup(struct snd_pcm_substream *substream)
{
	/* Each ADC3140 contributes four channels on its own serializer. */
	return snd_pcm_hw_constraint_minmax(substream->runtime,
					    SNDRV_PCM_HW_PARAM_CHANNELS, 1, 4);
}

/* See bela_gem_play_hw_params() for why the CPU masks are set here. */
static int bela_gem_adc_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);

	return snd_soc_dai_set_tdm_slot(cpu, 0, BELA_ADC_MASK,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

static const struct snd_soc_ops bela_gem_adc_ops = {
	.startup   = bela_gem_adc_startup,
	.hw_params = bela_gem_adc_hw_params,
};

static int bela_gem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *cpu_node, *aic_node, *dac_node, *adc1_node, *adc2_node;
	/*
	 * DSP_A (1-bit delay) + IB_NF is the framing validated on hardware and
	 * matches Bela's shipping McASP registers (XDATDLY=1, all polarity
	 * inversions clear -- davinci-mcasp's IB_NF clears ACLK[XR]POL). The
	 * ES9080Q decodes it as left-justified data on the negative edge of
	 * the 1-BCLK WS pulse.
	 *
	 * Clock roles are per component (dai_link_component.ext_fmt): the
	 * AIC3106 is the only BCLK/WCLK provider -- its PLL generates the
	 * 256-BCLK/8-slot frame (256-clock mode, set in prepare) and AUX_MCLK
	 * on GPIO1. The McASP, the ES9080Q and both ADC3140s are consumers.
	 * The ES9080Q requires an integer MCLK:BCLK ratio, which only holds
	 * because BCLK and AUX_MCLK derive from the one AIC3106 PLL. A single
	 * link-wide CBP_CFP would wrongly tell the ADC3140s to master the bus
	 * they share with the AIC3106.
	 */
	const unsigned int dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF;
	struct bela_gem_priv *priv;
	int ret;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	cpu_node  = of_parse_phandle(np, "cpu-dai",        0);
	aic_node  = of_parse_phandle(np, "aic3106-codec",  0);
	dac_node  = of_parse_phandle(np, "es9080q-codec",  0);
	adc1_node = of_parse_phandle(np, "adc3140-1-codec", 0);
	adc2_node = of_parse_phandle(np, "adc3140-2-codec", 0);

	if (!cpu_node || !aic_node || !dac_node || !adc1_node || !adc2_node) {
		dev_err(dev,
			"missing cpu-dai / aic3106-codec / es9080q-codec / adc3140-{1,2}-codec\n");
		ret = -EINVAL;
		goto err_put;
	}

	/*
	 * Link 0: McASP2 + { AIC3106, ES9080Q } (multi-codec).
	 * Capture is AIC3106-only (the ES9080Q has no capture stream and is
	 * skipped for that direction). Opening this link spins up the AIC3106 PLL
	 * -> AUX_MCLK, which the ES9080Q needs before its own init runs in the
	 * shared hw_params().
	 *
	 * Playback: ASoC does not intersect the codec channel ranges on a
	 * multi-codec link (snd_soc_runtime_calc_hw() uses the CPU DAI's range
	 * when num_codecs > 1), so the stream range comes from the McASP: both
	 * TX serializers x 8 slots = up to 16 channels, interleaved as described
	 * in the header comment. Each codec's own hw_params() is fixed up from
	 * its TDM slot mask (set in bela_gem_play_init()): the AIC3106 sees 2
	 * channels and the ES9080Q sees 8, whatever the stream width. The
	 * startup hook limits playback to {1, 2, 16}.
	 */
	priv->cpu[0].of_node		  = cpu_node;
	priv->cpu[0].ext_fmt		  = SND_SOC_DAIFMT_BC_FC;
	priv->platform[0].of_node	  = cpu_node;
	priv->play_codecs[0].of_node	  = aic_node;
	priv->play_codecs[0].dai_name	  = "tlv320aic3x-hifi";
	priv->play_codecs[0].ext_fmt	  = SND_SOC_DAIFMT_BP_FP;
	priv->play_codecs[1].of_node	  = dac_node;
	priv->play_codecs[1].dai_name	  = "es9080q-hifi";
	priv->play_codecs[1].ext_fmt	  = SND_SOC_DAIFMT_BC_FC;
	priv->dai_links[0].name		  = "bela-gem-play";
	priv->dai_links[0].stream_name	  = "Bela Gem Playback";
	priv->dai_links[0].cpus		  = &priv->cpu[0];
	priv->dai_links[0].num_cpus	  = 1;
	priv->dai_links[0].codecs	  = priv->play_codecs;
	priv->dai_links[0].num_codecs	  = ARRAY_SIZE(priv->play_codecs);
	priv->dai_links[0].platforms	  = &priv->platform[0];
	priv->dai_links[0].num_platforms  = 1;
	priv->dai_links[0].init		  = bela_gem_play_init;
	priv->dai_links[0].ops		  = &bela_gem_play_ops;
	priv->dai_links[0].dai_fmt	  = dai_fmt;

	/* Link 1: ADC3140 #1 (4-ch capture, clock consumer) */
	priv->cpu[1].of_node		  = cpu_node;
	priv->cpu[1].ext_fmt		  = SND_SOC_DAIFMT_BC_FC;
	priv->platform[1].of_node	  = cpu_node;
	priv->adc1_codec.of_node	  = adc1_node;
	priv->adc1_codec.dai_name	  = "tlv320adcx140-codec";
	priv->adc1_codec.ext_fmt	  = SND_SOC_DAIFMT_BC_FC;
	priv->dai_links[1].name		  = "bela-gem-adc1";
	priv->dai_links[1].stream_name	  = "Bela Gem ADC1";
	priv->dai_links[1].cpus		  = &priv->cpu[1];
	priv->dai_links[1].num_cpus	  = 1;
	priv->dai_links[1].codecs	  = &priv->adc1_codec;
	priv->dai_links[1].num_codecs	  = 1;
	priv->dai_links[1].platforms	  = &priv->platform[1];
	priv->dai_links[1].num_platforms  = 1;
	priv->dai_links[1].init		  = bela_gem_adc_init;
	priv->dai_links[1].ops		  = &bela_gem_adc_ops;
	priv->dai_links[1].dai_fmt	  = dai_fmt;
	priv->dai_links[1].capture_only	  = 1;

	/* Link 2: ADC3140 #2 (4-ch capture, clock consumer) */
	priv->cpu[2].of_node		  = cpu_node;
	priv->cpu[2].ext_fmt		  = SND_SOC_DAIFMT_BC_FC;
	priv->platform[2].of_node	  = cpu_node;
	priv->adc2_codec.of_node	  = adc2_node;
	priv->adc2_codec.dai_name	  = "tlv320adcx140-codec";
	priv->adc2_codec.ext_fmt	  = SND_SOC_DAIFMT_BC_FC;
	priv->dai_links[2].name		  = "bela-gem-adc2";
	priv->dai_links[2].stream_name	  = "Bela Gem ADC2";
	priv->dai_links[2].cpus		  = &priv->cpu[2];
	priv->dai_links[2].num_cpus	  = 1;
	priv->dai_links[2].codecs	  = &priv->adc2_codec;
	priv->dai_links[2].num_codecs	  = 1;
	priv->dai_links[2].platforms	  = &priv->platform[2];
	priv->dai_links[2].num_platforms  = 1;
	priv->dai_links[2].init		  = bela_gem_adc_init;
	priv->dai_links[2].ops		  = &bela_gem_adc_ops;
	priv->dai_links[2].dai_fmt	  = dai_fmt;
	priv->dai_links[2].capture_only	  = 1;

	/*
	 * The two TLV320ADC3140s are identical codecs, so they register
	 * controls/widgets with the same names ("Analog CH1 Mic Gain Volume",
	 * ...). Without unique prefixes the second one collides with the first
	 * and card registration fails with -EBUSY. Give each a name_prefix so
	 * its controls become "ADC1 ..." / "ADC2 ...".
	 */
	priv->codec_conf[0].dlc.of_node = adc1_node;
	priv->codec_conf[0].name_prefix = "ADC1";
	priv->codec_conf[1].dlc.of_node = adc2_node;
	priv->codec_conf[1].name_prefix = "ADC2";

	of_node_put(cpu_node);
	of_node_put(aic_node);
	of_node_put(dac_node);
	of_node_put(adc1_node);
	of_node_put(adc2_node);

	/* MCLK (audio_refclk1) — enable once; AIC PLL derives BCLK/WCLK. */
	priv->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->mclk))
		return dev_err_probe(dev, PTR_ERR(priv->mclk),
				     "failed to get mclk\n");

	priv->mclk_freq = clk_get_rate(priv->mclk);
	if (!priv->mclk_freq) {
		dev_warn(dev, "mclk rate is 0, defaulting to %u Hz\n",
			 BELA_MCLK_DEFAULT);
		priv->mclk_freq = BELA_MCLK_DEFAULT;
	}

	ret = clk_prepare_enable(priv->mclk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable mclk\n");

	priv->card.owner	    = THIS_MODULE;
	priv->card.dev		    = dev;
	priv->card.name		    = "BelaGemMulti";
	priv->card.dai_link	    = priv->dai_links;
	priv->card.num_links	    = ARRAY_SIZE(priv->dai_links);
	priv->card.codec_conf	    = priv->codec_conf;
	priv->card.num_configs	    = ARRAY_SIZE(priv->codec_conf);
	priv->card.dapm_widgets	    = bela_gem_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(bela_gem_widgets);
	priv->card.dapm_routes	    = bela_gem_routes;
	priv->card.num_dapm_routes  = ARRAY_SIZE(bela_gem_routes);
	priv->card.controls	    = bela_gem_controls;
	priv->card.num_controls	    = ARRAY_SIZE(bela_gem_controls);
	priv->card.fully_routed	    = true;

	snd_soc_of_parse_card_name(&priv->card, "model");
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(dev, &priv->card);
	if (ret) {
		clk_disable_unprepare(priv->mclk);
		return dev_err_probe(dev, ret, "failed to register card\n");
	}

	return 0;

err_put:
	of_node_put(cpu_node);
	of_node_put(aic_node);
	of_node_put(dac_node);
	of_node_put(adc1_node);
	of_node_put(adc2_node);
	return ret;
}

static void bela_gem_remove(struct platform_device *pdev)
{
	struct bela_gem_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->mclk);
}

static const struct of_device_id bela_gem_of_match[] = {
	{ .compatible = "ti,am625-bela-gem-multi" },
	{ }
};
MODULE_DEVICE_TABLE(of, bela_gem_of_match);

static struct platform_driver bela_gem_driver = {
	.probe  = bela_gem_probe,
	.remove = bela_gem_remove,
	.driver = {
		.name		= "am625-bela-gem",
		.of_match_table	= bela_gem_of_match,
	},
};
module_platform_driver(bela_gem_driver);

MODULE_DESCRIPTION("Bela Gem Multi ASoC machine driver for AM625");
MODULE_AUTHOR("Piyush Patle <piyushpatle1228@gmail.com>");
MODULE_AUTHOR("Giulio Moro <giuliomoro@gmail.com>");
MODULE_LICENSE("GPL");
