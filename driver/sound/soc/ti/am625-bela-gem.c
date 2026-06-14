// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bela Gem Multi Audio Cape — ASoC machine driver for AM625
 *
 * Four DAI links on a shared McASP2 TDM bus (8 slots x 32 bits, DSP_B).
 * McASP2 is the BCLK + WCLK (frame) master for the whole frame; every codec is
 * a clock consumer. (A stereo codec like the AIC3106 cannot master a frame
 * wider than its 2 channels, so the McASP clocks the 8-slot frame instead.)
 * Codecs still take their MCLK from audio_refclk1. Each codec uses its own
 * McASP serializer (data pin), starting at slot 0 of its own line:
 *
 *   Link 0: McASP2 <-> TLV320AIC3106  (stereo playback + capture, clock slave)
 *           AXR0 TX (slots 0-1), AXR1 RX (slots 0-1)
 *   Link 1: McASP2  -> ES9080Q        (8-ch playback, slave)
 *           AXR6 TX (slots 0-7)
 *   Link 2: McASP2  <- TLV320ADC3140 #1 (4-ch capture, slave)
 *           AXR4 RX (slots 0-3)
 *   Link 3: McASP2  <- TLV320ADC3140 #2 (4-ch capture, slave)
 *           AXR8 RX (slots 0-3)
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
 * McASP AUXCLK (high-frequency clock / AHCLKX source) rate used when the McASP
 * is the bit-clock master. The McASP divides this down to BCLK:
 *   24.576 MHz / 2 = 12.288 MHz = 256 * 48 kHz = 8 slots * 32 bits * 48 kHz.
 * The mcasp2 fck is pinned to this rate in the overlay (assigned-clock-rates),
 * and the same value is handed to the McASP via set_sysclk so it can program
 * the BCLK divider — without that, sysclk_freq stays 0 and the McASP never
 * generates a bit clock (silent DMA -EIO).
 */
#define BELA_MCASP_AUXCLK	24576000U

/*
 * Tell the McASP its AHCLKX rate as bit-clock master: clk_id 0 + CLOCK_OUT
 * selects AUXCLK (the internal functional clock) as HCLK and stores
 * sysclk_freq, which davinci-mcasp needs to compute the BCLK divider.
 */
static int bela_gem_set_mcasp_master_clk(struct snd_soc_dai *cpu)
{
	return snd_soc_dai_set_sysclk(cpu, 0, BELA_MCASP_AUXCLK,
				      SND_SOC_CLOCK_OUT);
}

struct bela_gem_priv {
	struct snd_soc_card		 card;
	struct snd_soc_dai_link		 dai_links[4];
	struct snd_soc_dai_link_component cpu[4];
	struct snd_soc_dai_link_component platform[4];
	struct snd_soc_dai_link_component aic_codec;
	struct snd_soc_dai_link_component dac_codec;
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

/* Link 0 init: McASP2 + AIC3106 — McASP masters BCLK/WCLK, AIC takes MCLK */
static int bela_gem_aic_init(struct snd_soc_pcm_runtime *rtd)
{
	struct bela_gem_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *aic = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	ret = bela_gem_set_mcasp_master_clk(cpu);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(cpu, BELA_AIC_MASK, BELA_AIC_MASK,
				       BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(aic, 0, priv->mclk_freq, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	return snd_soc_dai_set_tdm_slot(aic, BELA_AIC_MASK, BELA_AIC_MASK,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

/* Link 1 init: McASP2 + ES9080Q (playback, clock slave) */
static int bela_gem_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	struct bela_gem_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *dac = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	ret = bela_gem_set_mcasp_master_clk(cpu);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(cpu, BELA_DAC_MASK, 0,
				       BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(dac, 0, priv->mclk_freq, SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP)
		return ret;

	return snd_soc_dai_set_tdm_slot(dac, BELA_DAC_MASK, 0,
					BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
}

/* Link 2/3 init: McASP2 + TLV320ADC3140 (capture, clock slave) */
static int bela_gem_adc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *adc = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_tdm_slot(cpu, 0, BELA_ADC_MASK,
				       BELA_TDM_SLOTS, BELA_TDM_SLOT_WIDTH);
	if (ret)
		return ret;

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

static int bela_gem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *cpu_node, *aic_node, *dac_node, *adc1_node, *adc2_node;
	/*
	 * The McASP is the BCLK/WCLK + frame master for the whole 8-slot TDM
	 * frame; every codec is a clock consumer (CBC_CFC). The TLV320AIC3106
	 * cannot master a frame wider than its own 2 channels (mainline
	 * tlv320aic3x has no TDM-master frame-size control), so a stereo codec
	 * cannot clock the shared 8-slot frame the ES9080Q needs. The McASP
	 * generates a clean 256-BCLK/frame instead. Codecs still get their
	 * MCLK from audio_refclk1 (P2.11); only BCLK/WCLK move to the McASP.
	 */
	const unsigned int dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF |
				     SND_SOC_DAIFMT_CBC_CFC;
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

	/* Link 0: AIC3106 (clock master) */
	priv->cpu[0].of_node		  = cpu_node;
	priv->platform[0].of_node	  = cpu_node;
	priv->aic_codec.of_node		  = aic_node;
	priv->aic_codec.dai_name	  = "tlv320aic3x-hifi";
	priv->dai_links[0].name		  = "bela-gem-aic";
	priv->dai_links[0].stream_name	  = "Bela Gem Stereo";
	priv->dai_links[0].cpus		  = &priv->cpu[0];
	priv->dai_links[0].num_cpus	  = 1;
	priv->dai_links[0].codecs	  = &priv->aic_codec;
	priv->dai_links[0].num_codecs	  = 1;
	priv->dai_links[0].platforms	  = &priv->platform[0];
	priv->dai_links[0].num_platforms  = 1;
	priv->dai_links[0].init		  = bela_gem_aic_init;
	priv->dai_links[0].dai_fmt	  = dai_fmt;

	/* Link 1: ES9080Q (8-ch playback, slave) */
	priv->cpu[1].of_node		  = cpu_node;
	priv->platform[1].of_node	  = cpu_node;
	priv->dac_codec.of_node		  = dac_node;
	priv->dac_codec.dai_name	  = "es9080q-hifi";
	priv->dai_links[1].name		  = "bela-gem-dac";
	priv->dai_links[1].stream_name	  = "Bela Gem 8ch DAC";
	priv->dai_links[1].cpus		  = &priv->cpu[1];
	priv->dai_links[1].num_cpus	  = 1;
	priv->dai_links[1].codecs	  = &priv->dac_codec;
	priv->dai_links[1].num_codecs	  = 1;
	priv->dai_links[1].platforms	  = &priv->platform[1];
	priv->dai_links[1].num_platforms  = 1;
	priv->dai_links[1].init		  = bela_gem_dac_init;
	priv->dai_links[1].dai_fmt	  = dai_fmt;
	priv->dai_links[1].playback_only  = 1;

	/* Link 2: ADC3140 #1 (4-ch capture, slave) */
	priv->cpu[2].of_node		  = cpu_node;
	priv->platform[2].of_node	  = cpu_node;
	priv->adc1_codec.of_node	  = adc1_node;
	priv->adc1_codec.dai_name	  = "tlv320adcx140-codec";
	priv->dai_links[2].name		  = "bela-gem-adc1";
	priv->dai_links[2].stream_name	  = "Bela Gem ADC1";
	priv->dai_links[2].cpus		  = &priv->cpu[2];
	priv->dai_links[2].num_cpus	  = 1;
	priv->dai_links[2].codecs	  = &priv->adc1_codec;
	priv->dai_links[2].num_codecs	  = 1;
	priv->dai_links[2].platforms	  = &priv->platform[2];
	priv->dai_links[2].num_platforms  = 1;
	priv->dai_links[2].init		  = bela_gem_adc_init;
	priv->dai_links[2].dai_fmt	  = dai_fmt;
	priv->dai_links[2].capture_only	  = 1;

	/* Link 3: ADC3140 #2 (4-ch capture, slave) */
	priv->cpu[3].of_node		  = cpu_node;
	priv->platform[3].of_node	  = cpu_node;
	priv->adc2_codec.of_node	  = adc2_node;
	priv->adc2_codec.dai_name	  = "tlv320adcx140-codec";
	priv->dai_links[3].name		  = "bela-gem-adc2";
	priv->dai_links[3].stream_name	  = "Bela Gem ADC2";
	priv->dai_links[3].cpus		  = &priv->cpu[3];
	priv->dai_links[3].num_cpus	  = 1;
	priv->dai_links[3].codecs	  = &priv->adc2_codec;
	priv->dai_links[3].num_codecs	  = 1;
	priv->dai_links[3].platforms	  = &priv->platform[3];
	priv->dai_links[3].num_platforms  = 1;
	priv->dai_links[3].init		  = bela_gem_adc_init;
	priv->dai_links[3].dai_fmt	  = dai_fmt;
	priv->dai_links[3].capture_only	  = 1;

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
	cpu_node = aic_node = dac_node = adc1_node = adc2_node = NULL;

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
	snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
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
