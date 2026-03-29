// SPDX-License-Identifier: GPL-2.0
/*
 * Bela Audio Cape Machine Driver
 * Single sound card with TLV320AIC3104 + ES9080Q
 * 10 outputs (2 from AIC3104 + 8 from ES9080Q) + 2 inputs (from AIC3104)
 *
 * Clock architecture:
 *   AIC3104  --BCLK + FSYNC master (CBP_CFP)
 *   McASP    --BCLK + FSYNC slave  (CBC_CFC)
 *   ES9080Q  --BCLK + FSYNC slave  (CBC_CFC)
 *
 * TDM slot allocation (16 slots × 32 bits):
 *   Slots  0-1  : TLV320AIC3104 (tx_mask=0x0003, rx_mask=0x0003)
 *   Slots  2-9  : ES9080Q       (tx_mask=0x03FC)
 *   Slots 10-15 : Reserved
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#define TDM_SLOTS      16
#define TDM_SLOT_WIDTH 32
#define BELA_OUTPUTS   10
#define BELA_INPUTS    2

struct bela_priv {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codecs[2];  /* [0]=AIC3104, [1]=ES9080Q */
	struct snd_soc_dai_link_component platform;
	struct clk *mclk;
	unsigned int mclk_freq;   /* Currently programmed MCLK rate */
};

/* Sample rate helpers */

static const unsigned int bela_rates[] = {
	44100, 48000, 88200, 96000,
};

static const struct snd_pcm_hw_constraint_list bela_rate_constraints = {
	.count = ARRAY_SIZE(bela_rates),
	.list  = bela_rates,
};

/**
 * bela_get_mclk_rate - Return MCLK frequency appropriate for a sample rate
 * @rate: Sample rate in Hz
 *
 * 44.1 kHz family uses 22.5792 MHz; 48 kHz family uses 24.576 MHz.
 */
static unsigned int bela_get_mclk_rate(unsigned int rate)
{
	switch (rate) {
	case 44100:
	case 88200:
		return 22579200;
	case 48000:
	case 96000:
	default:
		return 24576000;
	}
}

/* PCM ops */

/**
 * bela_startup - Apply rate and channel constraints when a stream opens
 * @substream: Opening PCM substream
 *
 * Restricts sample rates to the supported list and caps channels to
 * BELA_OUTPUTS (playback) or BELA_INPUTS (capture).
 */
static int bela_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	ret = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &bela_rate_constraints);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = snd_pcm_hw_constraint_minmax(runtime,
						   SNDRV_PCM_HW_PARAM_CHANNELS,
						   1, BELA_OUTPUTS);
		if (ret < 0) {
			dev_err(rtd->dev,
				"Failed to set playback channel constraint: %d\n",
				ret);
			return ret;
		}
		dev_dbg(rtd->dev, "Playback: 1-%d channels\n", BELA_OUTPUTS);
	} else {
		ret = snd_pcm_hw_constraint_minmax(runtime,
						   SNDRV_PCM_HW_PARAM_CHANNELS,
						   1, BELA_INPUTS);
		if (ret < 0) {
			dev_err(rtd->dev,
				"Failed to set capture channel constraint: %d\n",
				ret);
			return ret;
		}
		dev_dbg(rtd->dev, "Capture: 1-%d channels\n", BELA_INPUTS);
	}

	return 0;
}

/**
 * bela_hw_params - Adjust MCLK rate when the sample-rate family changes
 * @substream: PCM substream being configured
 * @params: Requested hardware parameters
 *
 * All static DAI configuration (format, TDM slots, sysclk) is done once in
 * bela_init().  The only per-stream action needed here is switching the MCLK
 * oscillator between the 44.1 kHz family (22.5792 MHz) and the 48 kHz family
 * (24.576 MHz) when the application requests a different rate.
 *
 * If the rate family has not changed, this function is a no-op.
 */
static int bela_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct bela_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *aic3104_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int rate = params_rate(params);
	unsigned int mclk_freq = bela_get_mclk_rate(rate);
	int ret;

	if (!priv->mclk || mclk_freq == priv->mclk_freq)
		return 0;

	dev_dbg(card->dev, "Switching MCLK: %u -> %u Hz for rate %u\n",
		priv->mclk_freq, mclk_freq, rate);

	clk_disable_unprepare(priv->mclk);

	ret = clk_set_rate(priv->mclk, mclk_freq);
	if (ret) {
		dev_err(card->dev, "Failed to set MCLK rate %u: %d\n",
			mclk_freq, ret);
		return ret;
	}

	ret = clk_prepare_enable(priv->mclk);
	if (ret) {
		dev_err(card->dev, "Failed to re-enable MCLK: %d\n", ret);
		return ret;
	}

	priv->mclk_freq = mclk_freq;

	/* Inform AIC3104 of the new sysclk so its PLL can re-lock */
	ret = snd_soc_dai_set_sysclk(aic3104_dai, 0, mclk_freq,
				     SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP)
		dev_warn(card->dev,
			 "Failed to update AIC3104 sysclk: %d\n", ret);

	return 0;
}

static const struct snd_soc_ops bela_ops = {
	.startup   = bela_startup,
	.hw_params = bela_hw_params,
};

/* DAPM widgets, routes, and controls */

static const struct snd_soc_dapm_widget bela_dapm_widgets[] = {
	/* AIC3104 outputs */
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("AIC3104 Line Out", NULL),

	/* AIC3104 inputs */
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),

	/* ES9080Q board output connectors (channels 3-10) */
	SND_SOC_DAPM_OUTPUT("DAC Out 1"),
	SND_SOC_DAPM_OUTPUT("DAC Out 2"),
	SND_SOC_DAPM_OUTPUT("DAC Out 3"),
	SND_SOC_DAPM_OUTPUT("DAC Out 4"),
	SND_SOC_DAPM_OUTPUT("DAC Out 5"),
	SND_SOC_DAPM_OUTPUT("DAC Out 6"),
	SND_SOC_DAPM_OUTPUT("DAC Out 7"),
	SND_SOC_DAPM_OUTPUT("DAC Out 8"),
};

/*
 * DAPM routes
 *
 * Format: { sink, control, source }
 *
 * For ES9080Q: the machine-level "DAC Out N" board widgets connect to the
 * codec's "OUTN" SND_SOC_DAPM_OUTPUT pins defined in es9080q.c.  Routing to
 * the stream name "Playback" is incorrect --stream names are not widget names.
 */
static const struct snd_soc_dapm_route bela_dapm_routes[] = {
	/* AIC3104 output routing */
	{ "Headphone Jack",   NULL, "HPLOUT" },
	{ "Headphone Jack",   NULL, "HPROUT" },
	{ "AIC3104 Line Out", NULL, "LLOUT"  },
	{ "AIC3104 Line Out", NULL, "RLOUT"  },

	/* AIC3104 input routing */
	{ "LINE1L", NULL, "Line In" },
	{ "LINE1R", NULL, "Line In" },

	/*
	 * ES9080Q output routing
	 * "OUT1"-"OUT8" are the SND_SOC_DAPM_OUTPUT widget names in es9080q.c.
	 */
	{ "DAC Out 1", NULL, "OUT1" },
	{ "DAC Out 2", NULL, "OUT2" },
	{ "DAC Out 3", NULL, "OUT3" },
	{ "DAC Out 4", NULL, "OUT4" },
	{ "DAC Out 5", NULL, "OUT5" },
	{ "DAC Out 6", NULL, "OUT6" },
	{ "DAC Out 7", NULL, "OUT7" },
	{ "DAC Out 8", NULL, "OUT8" },
};

static const struct snd_kcontrol_new bela_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("AIC3104 Line Out"),
	SOC_DAPM_PIN_SWITCH("Line In"),
	SOC_DAPM_PIN_SWITCH("Mic In"),
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
 * DAI link init -- all static DAI configuration lives here
 *
 * Clock master assignment:
 *   AIC3104 is the BCLK/FSYNC master.  It generates 256 clocks per frame
 *   derived from its own MCLK PLL.  Both the McASP (CPU DAI) and the ES9080Q
 *   receive the clock passively.
 *
 *   McASP  (CPU DAI) = SND_SOC_DAIFMT_CBC_CFC  (clock+frame consumer/slave)
 *   AIC3104          = SND_SOC_DAIFMT_CBP_CFP  (clock+frame provider/master)
 *   ES9080Q          = SND_SOC_DAIFMT_CBC_CFC  (clock+frame consumer/slave)
 *
 * Called once by ASoC after all components are bound.
 */
static int bela_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct bela_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai     = snd_soc_rtd_to_cpu(rtd, 0);
	/* Access by index --matches the order in priv->codecs[] */
	struct snd_soc_dai *aic3104_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *es9080q_dai = snd_soc_rtd_to_codec(rtd, 1);
	int ret;

	dev_info(card->dev,
		 "Bela init: cpu=%s aic3104=%s es9080q=%s\n",
		 cpu_dai->name, aic3104_dai->name, es9080q_dai->name);

	/* 1. McASP (CPU DAI) -- clock slave, receives BCLK/FSYNC from AIC3104 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_DSP_B    |
				  SND_SOC_DAIFMT_IB_NF    |
				  SND_SOC_DAIFMT_CBC_CFC);   /* slave */
	if (ret) {
		dev_err(card->dev, "Failed to set CPU DAI format: %d\n", ret);
		return ret;
	}

	/* McASP: all 16 TDM slots, TX slots 0-9, RX slots 0-1 */
	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       0x03FF,  /* TX: slots 0-9 */
				       0x0003,  /* RX: slots 0-1 */
				       TDM_SLOTS, TDM_SLOT_WIDTH);
	if (ret) {
		dev_err(card->dev, "Failed to set CPU TDM slots: %d\n", ret);
		return ret;
	}

	/* 2. TLV320AIC3104 -- clock master, drives BCLK + FSYNC */
	ret = snd_soc_dai_set_fmt(aic3104_dai,
				  SND_SOC_DAIFMT_DSP_B    |
				  SND_SOC_DAIFMT_IB_NF    |
				  SND_SOC_DAIFMT_CBP_CFP);   /* master */
	if (ret) {
		dev_err(card->dev, "Failed to set AIC3104 format: %d\n", ret);
		return ret;
	}

	/* Supply MCLK to AIC3104 PLL */
	ret = snd_soc_dai_set_sysclk(aic3104_dai, 0, priv->mclk_freq,
				     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(card->dev, "Failed to set AIC3104 sysclk: %d\n", ret);
		return ret;
	}

	/* AIC3104 owns TDM slots 0-1 for both TX and RX */
	ret = snd_soc_dai_set_tdm_slot(aic3104_dai,
				       0x0003, 0x0003,
				       TDM_SLOTS, TDM_SLOT_WIDTH);
	if (ret) {
		dev_err(card->dev, "Failed to set AIC3104 TDM slots: %d\n",
			ret);
		return ret;
	}

	/* 3. ES9080Q -- clock slave, receives BCLK/FSYNC from AIC3104 */
	ret = snd_soc_dai_set_fmt(es9080q_dai,
				  SND_SOC_DAIFMT_DSP_B    |
				  SND_SOC_DAIFMT_IB_NF    |
				  SND_SOC_DAIFMT_CBC_CFC);   /* slave */
	if (ret) {
		dev_err(card->dev, "Failed to set ES9080Q format: %d\n", ret);
		return ret;
	}

	/*
	 * ES9080Q uses MCLK (not BCLK) as its sysclk reference for internal
	 * PLL and divider calculations --see es9080q_configure_clocking().
	 */
	ret = snd_soc_dai_set_sysclk(es9080q_dai, 0, priv->mclk_freq,
				     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(card->dev, "Failed to set ES9080Q sysclk: %d\n", ret);
		return ret;
	}

	/* ES9080Q owns TDM slots 2-9 for TX; no RX (DAC only) */
	ret = snd_soc_dai_set_tdm_slot(es9080q_dai,
				       0x03FC, 0x0000,
				       TDM_SLOTS, TDM_SLOT_WIDTH);
	if (ret) {
		dev_err(card->dev, "Failed to set ES9080Q TDM slots: %d\n",
			ret);
		return ret;
	}

	/* 4. Enable all board-level output pins by default */
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "Headphone Jack");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "AIC3104 Line Out");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "Line In");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "Mic In");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 1");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 2");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 3");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 4");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 5");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 6");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 7");
	snd_soc_dapm_enable_pin(snd_soc_card_to_dapm(card), "DAC Out 8");

	dev_info(card->dev,
		 "Bela init complete: AIC3104=master, McASP+ES9080Q=slave\n");
	dev_info(card->dev,
		 "TDM: AIC3104=slots 0-1, ES9080Q=slots 2-9, MCLK=%u Hz\n",
		 priv->mclk_freq);

	return snd_soc_dapm_sync(snd_soc_card_to_dapm(card));
}

/* Platform driver probe / remove */

/**
 * bela_probe - Initialise the Bela Audio Cape sound card
 * @pdev: Platform device
 *
 * Parses Device Tree, sets up the single multi-codec DAI link, enables MCLK,
 * and registers the sound card with ASoC.
 *
 * Node references obtained via of_parse_phandle() are released immediately
 * after being stored in the DAI link component structs; ASoC holds its own
 * reference from that point on.
 */
static int bela_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *cpu_node;
	struct device_node *aic3104_node;
	struct device_node *es9080q_node;
	struct bela_priv *priv;
	int ret;

	if (!np) {
		dev_err(dev, "Device tree node not found\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	/* Parse DT phandles */
	cpu_node     = of_parse_phandle(np, "cpu-dai",       0);
	aic3104_node = of_parse_phandle(np, "aic3104-codec", 0);
	es9080q_node = of_parse_phandle(np, "es9080q-codec", 0);

	if (!cpu_node || !aic3104_node || !es9080q_node) {
		ret = dev_err_probe(dev, -EINVAL,
				    "Missing cpu-dai, aic3104-codec or es9080q-codec in DT\n");
		goto err_put_nodes;
	}

	/* Store of_node pointers --ASoC will take its own reference */
	priv->cpu.of_node       = cpu_node;
	priv->platform.of_node  = cpu_node;
	priv->codecs[0].of_node = aic3104_node;
	priv->codecs[0].dai_name = "tlv320aic3x-hifi";
	priv->codecs[1].of_node = es9080q_node;
	priv->codecs[1].dai_name = "es9080q-hifi";

	/*
	 * Release our references now --ASoC holds its own from this point.
	 * Nulling the locals prevents double-put in the error path.
	 */
	of_node_put(cpu_node);
	of_node_put(aic3104_node);
	of_node_put(es9080q_node);
	cpu_node = aic3104_node = es9080q_node = NULL;

	/* Get MCLK; required by bela,audio-cape binding */
	priv->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->mclk))
		return dev_err_probe(dev, PTR_ERR(priv->mclk),
				     "Failed to get MCLK\n");

	priv->mclk_freq = bela_get_mclk_rate(48000);
	ret = clk_set_rate(priv->mclk, priv->mclk_freq);
	if (ret)
		dev_warn(dev, "Failed to set initial MCLK rate: %d\n", ret);

	ret = clk_prepare_enable(priv->mclk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable MCLK\n");

	/* DAI link --single link with two codecs on shared TDM bus */
	priv->dai_link.name        = "Bela Unified TDM";
	priv->dai_link.stream_name = "Bela Multi-Channel Audio";
	priv->dai_link.cpus        = &priv->cpu;
	priv->dai_link.num_cpus    = 1;
	priv->dai_link.codecs      = priv->codecs;
	priv->dai_link.num_codecs  = 2;
	priv->dai_link.platforms   = &priv->platform;
	priv->dai_link.num_platforms = 1;
	priv->dai_link.ops         = &bela_ops;
	priv->dai_link.init        = bela_init;
	/*
	 * dai_fmt hint for ASoC: AIC3104 is clock provider.
	 * Individual DAIs are configured explicitly in bela_init().
	 */
	priv->dai_link.dai_fmt = SND_SOC_DAIFMT_DSP_B   |
				 SND_SOC_DAIFMT_IB_NF    |
				 SND_SOC_DAIFMT_CBP_CFP;

	/* Sound card */
	priv->card.owner            = THIS_MODULE;
	priv->card.dev              = dev;
	priv->card.name             = "Bela";
	priv->card.long_name        = "Bela Audio Cape - 10+2 Channel";
	priv->card.dai_link         = &priv->dai_link;
	priv->card.num_links        = 1;
	priv->card.dapm_widgets     = bela_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(bela_dapm_widgets);
	priv->card.dapm_routes      = bela_dapm_routes;
	priv->card.num_dapm_routes  = ARRAY_SIZE(bela_dapm_routes);
	priv->card.controls         = bela_controls;
	priv->card.num_controls     = ARRAY_SIZE(bela_controls);
	priv->card.fully_routed     = true;

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret && ret != -EINVAL)
		dev_warn(dev, "Failed to parse card model: %d\n", ret);

	ret = snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
	if (ret && ret != -EINVAL)
		dev_warn(dev, "Failed to parse audio-routing: %d\n", ret);

	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(dev, &priv->card);
	if (ret) {
		clk_disable_unprepare(priv->mclk);
		return dev_err_probe(dev, ret,
				     "Failed to register sound card\n");
	}

	dev_info(dev, "Bela sound card registered (%d outputs + %d inputs)\n",
		 BELA_OUTPUTS, BELA_INPUTS);
	dev_info(dev, "Clock master: AIC3104  MCLK: %u Hz\n", priv->mclk_freq);

	return 0;

err_put_nodes:
	of_node_put(cpu_node);
	of_node_put(aic3104_node);
	of_node_put(es9080q_node);
	return ret;
}

/**
 * bela_remove - Clean up on driver removal
 * @pdev: Platform device being removed
 *
 * devm_ resources (card registration, allocations, clk_get) are freed
 * automatically.  The only manual action is disabling MCLK, which was enabled
 * without devm_ in probe().
 */
static void bela_remove(struct platform_device *pdev)
{
	struct bela_priv *priv = platform_get_drvdata(pdev);

	/*
	 * devm_snd_soc_register_card is already torn down before remove()
	 * is called, so all streams are already stopped.  Just gate the clock.
	 */
	if (priv && priv->mclk)
		clk_disable_unprepare(priv->mclk);

	dev_info(&pdev->dev, "Bela sound card removed\n");
}

/* Module boilerplate */

static const struct of_device_id bela_of_match[] = {
	{ .compatible = "bela,audio-cape" },
	{ }
};
MODULE_DEVICE_TABLE(of, bela_of_match);

static struct platform_driver bela_driver = {
	.probe      = bela_probe,
	.remove     = bela_remove,
	.driver = {
		.name           = "bela-audio-cape",
		.of_match_table = bela_of_match,
	},
};
module_platform_driver(bela_driver);

MODULE_DESCRIPTION("Bela Audio Cape Machine Driver (TLV320AIC3104 + ES9080Q)");
MODULE_AUTHOR("Jian De <jiande2020@gmail.com>");
MODULE_AUTHOR("Piyush Patle <piyushpatle1228@gmail.com>");
MODULE_AUTHOR("Giulio Moro");
MODULE_LICENSE("GPL");
