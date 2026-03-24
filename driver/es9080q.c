// SPDX-License-Identifier: GPL-2.0-only
/*
 * es9080q.c  --  ES9080Q ALSA SoC Audio driver
 */

#include <linux/init.h>
#include <linux/log2.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <linux/regulator/consumer.h>

/* ES9080Q constants from C++ implementation */
#define ES9080Q_NUM_BITS         256
#define ES9080Q_SLOT_SIZE        32
#define ES9080Q_DATA_SIZE        16
#define ES9080Q_STARTING_SLOT    0
#define ES9080Q_NUM_SLOTS        (ES9080Q_NUM_BITS / ES9080Q_SLOT_SIZE)
#define ES9080Q_NUM_OUT_CHANNELS 8

/* Volume control constants */
#define ES9080Q_VOLUME_MIN  0
#define ES9080Q_VOLUME_MAX  255
#define ES9080Q_VOLUME_STEP 1

/* Register definitions */
#define ES9080Q_REG_AMP_CTRL        0     /* Turn on/off AMP */
#define ES9080Q_REG_CLK_EN          1     /* Clock enable */
#define ES9080Q_REG_TDM_EN          2     /* TDM enable */
#define ES9080Q_REG_DAC_CONFIG      3     /* Sample rate config */
#define ES9080Q_REG_MASTER_CLK      4     /* Master clock config */
#define ES9080Q_REG_ANALOG_CTRL     5     /* Analog section control */
#define ES9080Q_REG_CP_CLOCK_DIV    6     /* Charge pump clock divider */
#define ES9080Q_REG_ANALOG_DELAY    7     /* Analog delay sequence */
#define ES9080Q_REG_PLL_LOCK        51    /* PLL lock signal */
#define ES9080Q_REG_INPUT_CONFIG    77    /* Input configuration */
#define ES9080Q_REG_MASTER_MODE     78    /* Master mode config */
#define ES9080Q_REG_TDM_CONFIG1     79    /* TDM configuration 1 */
#define ES9080Q_REG_TDM_CONFIG2     80    /* TDM configuration 2 */
#define ES9080Q_REG_TDM_CONFIG3     81    /* TDM configuration 3 */
#define ES9080Q_REG_BCK_WS_MONITOR  82    /* BCK/WS monitor */
#define ES9080Q_REG_TDM_VALID_PULSE 83    /* TDM valid pulse config */
#define ES9080Q_REG_DAC_RESYNC      92    /* DAC clock resync */
#define ES9080Q_REG_VOLUME_BASE     94    /* Volume registers start (94-101) */
#define ES9080Q_REG_VOLUME_CTRL     105   /* Volume control */
#define ES9080Q_REG_FILTER_SHAPE    108   /* Filter shape */
#define ES9080Q_REG_DITHER          109   /* Dither control */
#define ES9080Q_REG_THD_C2_135      111   /* THD compensation CH1/3/5/7 */
#define ES9080Q_REG_THD_C2_246      115   /* THD compensation CH2/4/6/8 */
#define ES9080Q_REG_AUTOMUTE        119   /* Automute control */
#define ES9080Q_REG_NSMOD_PHASE     128   /* NSMOD dither phases */
#define ES9080Q_REG_NSMOD_TYPE      129   /* NSMOD dither type */
#define ES9080Q_REG_GAIN_18DB       154   /* 18dB gain boost */

/* Write-only registers */
#define ES9080Q_REG_RESET_PLL  192   /* Reset and PLL register 1 */
#define ES9080Q_REG_GPIO_PLL   193   /* GPIO and PLL register 2 */
#define ES9080Q_REG_PLL_PARAM  202   /* PLL parameters */

/* THD Compensation Continuation Registers */
#define ES9080Q_REG_THD_C2_135_CONT 112   /* THD C2 coefficient continued (CH1/3/5/7) */
#define ES9080Q_REG_THD_C3_135      113   /* THD C3 coefficient (CH1/3/5/7) */
#define ES9080Q_REG_THD_C2_246_CONT 116   /* THD C2 coefficient continued (CH2/4/6/8) */
#define ES9080Q_REG_THD_C3_246      117   /* THD C3 coefficient (CH2/4/6/8) */

/* NSMOD Quantizer Registers */
#define ES9080Q_REG_NSMOD_CH12_QUANT 131  /* NSMOD quantizer CH1/2 */
#define ES9080Q_REG_NSMOD_CH34_QUANT 132  /* NSMOD quantizer CH3/4 */
#define ES9080Q_REG_NSMOD_CH56_QUANT 133  /* NSMOD quantizer CH5/6 */
#define ES9080Q_REG_NSMOD_CH78_QUANT 134  /* NSMOD quantizer CH7/8 */

/* DRE Register */
#define ES9080Q_REG_DRE 136   /* Dynamic Range Enhancement */

/* TDM Channel Configuration Base */
#define ES9080Q_REG_TDM_CH_BASE 84    /* TDM channel configuration base */

/* Register Values - Control Bits */
#define ES9080Q_AMP_CTRL_MUTE   0x00  /* Amplifier muted */
#define ES9080Q_AMP_CTRL_UNMUTE 0x02  /* Amplifier unmuted */
#define ES9080Q_CLK_EN_ALL      0xFF  /* All clocks enabled */
#define ES9080Q_TDM_EN_MODE1    0x01  /* TDM mode 1 (standard) */
#define ES9080Q_TDM_LJ_MODE_EN  0x80  /* Left-justified mode enable */

/* THD Coefficient Values */
#define ES9080Q_THD_C2_CONT_VAL 0x01  /* THD C2 continuation value */
#define ES9080Q_THD_C3_VAL      0x8D  /* THD C3 coefficient value */
#define ES9080Q_NSMOD_QUANT_VAL 0x44  /* NSMOD quantizer default value */
#define ES9080Q_DRE_DEFAULT     0x00  /* DRE disabled by default */

/* Filter and Dither Values */
#define ES9080Q_FILTER_SHAPE_VAL 0x46  /* Filter shape configuration */
#define ES9080Q_DITHER_VAL       0xE4  /* Dither control value */

/* Timing Constants (microseconds/milliseconds) */
#define ES9080Q_RESET_ASSERT_US  500   /* Reset assertion delay */
#define ES9080Q_RESET_RELEASE_US 100   /* Reset release delay */
#define ES9080Q_RETRY_DELAY_US   100   /* I2C retry delay */
#define ES9080Q_BCLK_STARTUP_MS  50    /* BCLK startup stabilization time */
#define ES9080Q_RESYNC_DELAY_US  10    /* DAC resync sequence delay */

/* DAC Resync Values */
#define ES9080Q_DAC_RESYNC_START  0x10  /* Start DAC resync */
#define ES9080Q_DAC_RESYNC_ACTIVE 0x0F  /* Active resync state */
#define ES9080Q_DAC_RESYNC_END    0x00  /* End DAC resync */

/* TDM Configuration Values */
#define ES9080Q_TDM_LJ_MODE_BITS   7   /* Left-justified mode bit position */
#define ES9080Q_TDM_VALID_EDGE_NEG 0   /* Negative edge for valid pulse */
#define ES9080Q_TDM_VALID_PULSE_8  8   /* Valid pulse length for 8+ channels */

/* Supported sample rates */
#define ES9080Q_RATES (SNDRV_PCM_RATE_44100  | SNDRV_PCM_RATE_48000  | \
		       SNDRV_PCM_RATE_88200  | SNDRV_PCM_RATE_96000  | \
		       SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)

struct es9080q_priv {
	struct i2c_client *rw_client;    /* Read/write client (0x48) */
	struct i2c_client *wo_client;    /* Write-only client (0x4c) */
	struct regmap *rw_regmap;
	struct regmap *wo_regmap;
	struct gpio_desc *reset_gpio;
	bool codec_initialized;
	bool is_master;
	unsigned int sysclk_freq;
	int volume[ES9080Q_NUM_OUT_CHANNELS];  /* Volume per channel */
	/* TDM slot configuration stored from set_tdm_slot */
	unsigned int tdm_tx_mask;
	int tdm_slots;
	int tdm_slot_width;
	int tdm_first_slot;
};

/* Initial register sequence for BCLK-dependent initialization */
static const struct reg_sequence es9080q_init_seq[] = {
	/* Set GPIO1 (MCLK) pad to input mode, invert CLKHV phase for better DNR */
	{ ES9080Q_REG_RESET_PLL, 0x03 },
	/* PLL bypass, remove 10k DVDD shunt, set PLL input to MCLK, enable PLL inputs */
	{ ES9080Q_REG_GPIO_PLL, 0xC3 },
	/* PLL parameters */
	{ ES9080Q_REG_PLL_PARAM, 0x40 },
};

/*
 * Register defaults table — populated from datasheet Power-On Reset values.
 * Only the registers actively used by this driver are listed; hardware defaults
 * for all others are 0x00 unless the datasheet states otherwise.
 */
static const struct reg_default es9080q_reg_defaults[] = {
	{ ES9080Q_REG_AMP_CTRL,        0x00 },
	{ ES9080Q_REG_CLK_EN,          0x00 },
	{ ES9080Q_REG_TDM_EN,          0x00 },
	{ ES9080Q_REG_DAC_CONFIG,      0x00 },
	{ ES9080Q_REG_MASTER_CLK,      0x00 },
	{ ES9080Q_REG_ANALOG_CTRL,     0x00 },
	{ ES9080Q_REG_CP_CLOCK_DIV,    0x00 },
	{ ES9080Q_REG_ANALOG_DELAY,    0x00 },
	{ ES9080Q_REG_PLL_LOCK,        0x00 },
	{ ES9080Q_REG_INPUT_CONFIG,    0x00 },
	{ ES9080Q_REG_MASTER_MODE,     0x00 },
	{ ES9080Q_REG_TDM_CONFIG1,     0x00 },
	{ ES9080Q_REG_TDM_CONFIG2,     0x00 },
	{ ES9080Q_REG_TDM_CONFIG3,     0x00 },
	{ ES9080Q_REG_BCK_WS_MONITOR,  0x00 },
	{ ES9080Q_REG_TDM_VALID_PULSE, 0x00 },
	{ ES9080Q_REG_DAC_RESYNC,      0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 0, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 1, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 2, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 3, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 4, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 5, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 6, 0x00 },
	{ ES9080Q_REG_VOLUME_BASE + 7, 0x00 },
	{ ES9080Q_REG_VOLUME_CTRL,     0x00 },
	{ ES9080Q_REG_FILTER_SHAPE,    0x00 },
	{ ES9080Q_REG_DITHER,          0x00 },
	{ ES9080Q_REG_THD_C2_135,      0x00 },
	{ ES9080Q_REG_THD_C2_135_CONT, 0x00 },
	{ ES9080Q_REG_THD_C3_135,      0x00 },
	{ ES9080Q_REG_THD_C2_246,      0x00 },
	{ ES9080Q_REG_THD_C2_246_CONT, 0x00 },
	{ ES9080Q_REG_THD_C3_246,      0x00 },
	{ ES9080Q_REG_AUTOMUTE,        0x00 },
	{ ES9080Q_REG_NSMOD_PHASE,     0x00 },
	{ ES9080Q_REG_NSMOD_TYPE,      0x00 },
	{ ES9080Q_REG_NSMOD_CH12_QUANT, 0x00 },
	{ ES9080Q_REG_NSMOD_CH34_QUANT, 0x00 },
	{ ES9080Q_REG_NSMOD_CH56_QUANT, 0x00 },
	{ ES9080Q_REG_NSMOD_CH78_QUANT, 0x00 },
	{ ES9080Q_REG_DRE,             0x00 },
};

/*
 * es9080q_wo_write - Write to a write-only register with retries
 *
 * ES9080Q exposes registers 192-203 via a separate I2C address that is
 * write-only (no readback). This wrapper retries up to 3 times with a
 * short inter-retry delay to absorb transient I2C bus errors.
 */
static int es9080q_wo_write(struct es9080q_priv *priv, unsigned int reg,
			    unsigned int val)
{
	struct device *dev = &priv->rw_client->dev;
	int ret;
	int retry;

	for (retry = 0; retry < 3; retry++) {
		ret = regmap_write(priv->wo_regmap, reg, val);
		if (ret == 0) {
			dev_dbg(dev, "WO write reg 0x%02X = 0x%02X\n", reg, val);
			return 0;
		}

		dev_warn(dev, "WO write failed reg 0x%02X: %d (attempt %d)\n",
			 reg, ret, retry + 1);

		if (retry < 2)
			udelay(ES9080Q_RETRY_DELAY_US);
	}

	return ret;
}

/*
 * es9080q_rw_write - Write to a read/write register with retries
 *
 * ES9080Q I2C interface sometimes experiences transient write failures,
 * particularly during codec configuration in the presence of other I2C traffic.
 * Retries up to 3 times with ES9080Q_RETRY_DELAY_US between attempts.
 */
static int es9080q_rw_write(struct es9080q_priv *priv, unsigned int reg,
			    unsigned int val)
{
	struct device *dev = &priv->rw_client->dev;
	int ret;
	int retry;

	for (retry = 0; retry < 3; retry++) {
		ret = regmap_write(priv->rw_regmap, reg, val);
		if (ret == 0) {
			dev_dbg(dev, "RW write reg 0x%02X = 0x%02X\n", reg, val);
			return 0;
		}

		dev_warn(dev, "RW write failed reg 0x%02X: %d (attempt %d)\n",
			 reg, ret, retry + 1);

		if (retry < 2)
			udelay(ES9080Q_RETRY_DELAY_US);
	}

	return ret;
}

/**
 * es9080q_configure_clocking - Configure ES9080Q clocking parameters
 * @priv: ES9080Q private driver data
 * @sample_rate: Audio sample rate in Hz (44.1kHz, 48kHz, etc.)
 *
 * The ES9080Q requires precise clock configuration with the relationship:
 * MCLK = sample_rate * 256 (or other factors based on slot size)
 * Dividers must be calculated to achieve correct clock ratios.
 *
 * This function:
 * 1. Calculates Master Clock (MENC) divider from MCLK frequency
 * 2. Configures Word Select (WS) scaling for proper frame synchronization
 * 3. Sets Charge Pump clock within 500kHz-1MHz range for PLL stability
 * 4. For master mode, configures BCK and WS generation
 *
 * Returns 0 on success, negative error code on failure
 */
static int es9080q_configure_clocking(struct es9080q_priv *priv,
				      unsigned int sample_rate)
{
	/* All declarations at top of function — C89 kernel style */
	unsigned int mclk_freq = priv->sysclk_freq;
	const unsigned int mclk_over_bck = 2;  /* MCLK/BCK ratio */
	const unsigned int mclk_over_ws = ES9080Q_NUM_BITS * mclk_over_bck;
	const bool is16bit = (ES9080Q_SLOT_SIZE == 16);
	const unsigned int master_bck_div1 = 1;
	unsigned int divide_value_menc;
	unsigned int ws_scale_factor;
	unsigned int master_ws_scale;
	unsigned int select_idac_num;
	unsigned int select_menc_num;
	unsigned int cp_clk_div;
	unsigned int cp_clock;
	unsigned int master_frame_length;
	const unsigned int cp_clock_min = 500000;
	const unsigned int cp_clock_max = 1000000;
	int ret;

	dev_dbg(&priv->rw_client->dev,
		"Configuring clocking: MCLK=%u, Fs=%u\n",
		mclk_freq, sample_rate);

	divide_value_menc = mclk_over_bck /
			    (master_bck_div1 ? 1 : (is16bit ? 4 : 2));
	ws_scale_factor   = mclk_over_ws / (divide_value_menc * 128);
	if (!is_power_of_2(ws_scale_factor)) {
		dev_err(&priv->rw_client->dev,
			"Invalid WS scale factor %u\n", ws_scale_factor);
		return -EINVAL;
	}

	master_ws_scale = ilog2(ws_scale_factor);

	if (master_ws_scale > 4) {
		dev_err(&priv->rw_client->dev,
			"Invalid WS scale factor %u\n", ws_scale_factor);
		return -EINVAL;
	}

	/* DAC CONFIG - Sample Rate register */
	select_idac_num = mclk_over_ws / 128 - 1;
	ret = es9080q_rw_write(priv, ES9080Q_REG_DAC_CONFIG, select_idac_num);
	if (ret)
		return ret;

	/* MASTER CLOCK CONFIG */
	if (divide_value_menc < 1 || divide_value_menc > 128) {
		dev_err(&priv->rw_client->dev,
			"Invalid MENC divider %u\n", divide_value_menc);
		return -EINVAL;
	}

	select_menc_num = divide_value_menc - 1;
	ret = es9080q_rw_write(priv, ES9080Q_REG_MASTER_CLK, select_menc_num);
	if (ret)
		return ret;

	/* CP CLOCK DIV - Charge pump clock must stay within 500kHz–1MHz */
	cp_clock = 0;
	for (cp_clk_div = 0; cp_clk_div <= 255; cp_clk_div++) {
		cp_clock = mclk_freq / ((cp_clk_div + 1) * 2);
		if (cp_clock <= cp_clock_max)
			break;
	}

	if (cp_clock < cp_clock_min || cp_clock > cp_clock_max) {
		dev_err(&priv->rw_client->dev,
			"Cannot find valid CP_CLK_DIV\n");
		return -EINVAL;
	}

	ret = es9080q_rw_write(priv, ES9080Q_REG_CP_CLOCK_DIV, cp_clk_div);
	if (ret)
		return ret;

	/* Configure TDM clocking registers only in master mode */
	if (priv->is_master) {
		master_frame_length = is16bit ? 2 : 0;
		ret = es9080q_rw_write(priv, ES9080Q_REG_MASTER_MODE,
				       (master_bck_div1 << 6) |
				       (master_frame_length << 3) |
				       (1 << 2));  /* Pulse WS mode */
		if (ret)
			return ret;

		ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_CONFIG1,
				       (master_ws_scale << 4) |
				       (ES9080Q_NUM_SLOTS - 1));
		if (ret)
			return ret;
	}

	return 0;
}

/* Initialize ES9080Q codec once BCLK is stable */
static int es9080q_initialize_codec(struct es9080q_priv *priv,
				    unsigned int sample_rate)
{
	/* All declarations at top — C89 kernel style */
	unsigned int tdm_bit_width;
	int slot;
	int ret;
	int i;

	if (priv->codec_initialized) {
		dev_dbg(&priv->rw_client->dev, "Codec already initialized\n");
		return 0;
	}

	dev_dbg(&priv->rw_client->dev, "Initializing ES9080Q\n");

	/* Enable clocks and TDM */
	ret = es9080q_rw_write(priv, ES9080Q_REG_CLK_EN, ES9080Q_CLK_EN_ALL);
	if (ret)
		return ret;

	ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_EN, ES9080Q_TDM_EN_MODE1);
	if (ret)
		return ret;

	/* Configure clocking */
	ret = es9080q_configure_clocking(priv, sample_rate);
	if (ret)
		return ret;

	/* Enable analog section */
	ret = es9080q_rw_write(priv, ES9080Q_REG_ANALOG_CTRL, 0xFF);
	if (ret)
		return ret;

	/* Analog delay sequence */
	ret = es9080q_rw_write(priv, ES9080Q_REG_ANALOG_DELAY, 0xBB);
	if (ret)
		return ret;

	/* Force PLL lock signal */
	ret = es9080q_rw_write(priv, ES9080Q_REG_PLL_LOCK, 0x80);
	if (ret)
		return ret;

	/* INPUT CONFIG */
	ret = es9080q_rw_write(priv, ES9080Q_REG_INPUT_CONFIG,
			       (priv->is_master << 4) |  /* master mode */
			       (0 << 2) |                /* INPUT_SEL: TDM */
			       (0 << 0));                /* AUTO_INPUT_SELECT: off */
	if (ret)
		return ret;

	/* TDM CONFIG2 - Left justified mode */
	ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_CONFIG2,
			       (1 << ES9080Q_TDM_LJ_MODE_BITS) |
			       (ES9080Q_TDM_VALID_EDGE_NEG << 6) |
			       (ES9080Q_TDM_VALID_PULSE_8 << 0));
	if (ret)
		return ret;

	/*
	 * TDM_CONFIG3 register: Configures TDM bit width and protocol options
	 * Bits [7:6]: TDM_BIT_WIDTH - slot size (0=32bit, 1=24bit, 2=16bit)
	 * Bit  [5]:  TDM_CHAIN_MODE - 0 to disable daisy-chain
	 * Bit  [0]:  TDM_DATA_LATCH_ADJ - 0 for default latch adjustment
	 */
	switch (ES9080Q_SLOT_SIZE) {
	case 32:
		tdm_bit_width = 0;
		break;
	case 24:
		tdm_bit_width = 1;
		break;
	case 16:
		tdm_bit_width = 2;
		break;
	default:
		dev_err(&priv->rw_client->dev,
			"Invalid slot size %d\n", ES9080Q_SLOT_SIZE);
		return -EINVAL;
	}

	ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_CONFIG3,
			       (tdm_bit_width << 6) |
			       (0 << 5) |   /* TDM_CHAIN_MODE: disable */
			       (0 << 0));   /* TDM_DATA_LATCH_ADJ: default */
	if (ret)
		return ret;

	/* BCK/WS Monitor - disable */
	ret = es9080q_rw_write(priv, ES9080Q_REG_BCK_WS_MONITOR, 0x00);
	if (ret)
		return ret;

	/* TDM Valid Pulse Config */
	ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_VALID_PULSE, 0x00);
	if (ret)
		return ret;

	/*
	 * Configure TDM channel mapping (registers 84-91 map to channels 0-7).
	 *
	 * ES9080Q architecture in shared TDM bus:
	 * - Bela Audio Cape uses 16 TDM slots total (McASP/McBSP format)
	 * - TLV320AIC3104 occupies slots 0-1 (2-in/2-out codec)
	 * - ES9080Q occupies slots 2-9 (8 independent audio output channels)
	 * - Slots 10-15 unused
	 *
	 * TDM_CHx registers (84-91) configure each output channel:
	 * - Bit  [7]:  TDM_VALID_PULSE_POS_MSB - only used in reg 84 (ch 0)
	 * - Bits [6:4]: TDM_DATA_LINE_SELECT - which TDM data line (0 = line 1)
	 * - Bits [3:0]: SLOT_SELECT - which TDM slot number (0-15)
	 *
	 * Use the first slot stored by set_tdm_slot so the machine driver can
	 * override the starting slot at runtime (e.g. slot 2 for Bela cape).
	 */
	slot = priv->tdm_first_slot;
	for (i = 0; i < ES9080Q_NUM_OUT_CHANNELS; i++, slot++) {
		ret = es9080q_rw_write(priv, ES9080Q_REG_TDM_CH_BASE + i,
				       ((i == 0) ? (0 << 7) : 0) | /* VALID_PULSE_POS_MSB */
				       (0 << 4) |                   /* data line 1 */
				       (slot << 0));                 /* slot number */
		if (ret)
			return ret;
	}

	/* Filter shape and dither */
	ret = es9080q_rw_write(priv, ES9080Q_REG_FILTER_SHAPE,
			       ES9080Q_FILTER_SHAPE_VAL);
	if (ret)
		return ret;

	ret = es9080q_rw_write(priv, ES9080Q_REG_DITHER, ES9080Q_DITHER_VAL);
	if (ret)
		return ret;

	/* THD Compensation registers */
	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C2_135, 0x68);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C2_135_CONT,
			       ES9080Q_THD_C2_CONT_VAL);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C3_135,
			       ES9080Q_THD_C3_VAL);
	if (ret)
		return ret;

	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C2_246, 0x68);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C2_246_CONT,
			       ES9080Q_THD_C2_CONT_VAL);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_THD_C3_246,
			       ES9080Q_THD_C3_VAL);
	if (ret)
		return ret;

	/* Automute - disable for all channels */
	ret = es9080q_rw_write(priv, ES9080Q_REG_AUTOMUTE, 0x00);
	if (ret)
		return ret;

	/* NSMOD registers */
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_PHASE, 0xCC);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_TYPE, 0x54);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_CH12_QUANT,
			       ES9080Q_NSMOD_QUANT_VAL);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_CH34_QUANT,
			       ES9080Q_NSMOD_QUANT_VAL);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_CH56_QUANT,
			       ES9080Q_NSMOD_QUANT_VAL);
	if (ret)
		return ret;
	ret = es9080q_rw_write(priv, ES9080Q_REG_NSMOD_CH78_QUANT,
			       ES9080Q_NSMOD_QUANT_VAL);
	if (ret)
		return ret;

	/* DRE register */
	ret = es9080q_rw_write(priv, ES9080Q_REG_DRE, ES9080Q_DRE_DEFAULT);
	if (ret)
		return ret;

	/* Volume control setup */
	ret = es9080q_rw_write(priv, ES9080Q_REG_VOLUME_CTRL,
			       (1 << 6) |    /* FORCE_VOLUME: immediate update */
			       (0 << 5) |    /* Separated volume control */
			       (0 << 4));    /* Per-channel volume control */
	if (ret)
		return ret;

	/* Initialize per-channel volumes */
	for (i = 0; i < ES9080Q_NUM_OUT_CHANNELS; i++) {
		ret = es9080q_rw_write(priv, ES9080Q_REG_VOLUME_BASE + i,
				       priv->volume[i]);
		if (ret)
			return ret;
	}

	/* DAC resync sequence */
	ret = es9080q_rw_write(priv, ES9080Q_REG_DAC_RESYNC,
			       ES9080Q_DAC_RESYNC_START);
	if (ret)
		return ret;
	udelay(ES9080Q_RESYNC_DELAY_US);

	ret = es9080q_rw_write(priv, ES9080Q_REG_DAC_RESYNC,
			       ES9080Q_DAC_RESYNC_ACTIVE);
	if (ret)
		return ret;
	udelay(ES9080Q_RESYNC_DELAY_US);

	ret = es9080q_rw_write(priv, ES9080Q_REG_DAC_RESYNC,
			       ES9080Q_DAC_RESYNC_END);
	if (ret)
		return ret;

	/* Turn on the AMP */
	ret = es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
			       ES9080Q_AMP_CTRL_UNMUTE);
	if (ret)
		return ret;

	priv->codec_initialized = true;
	dev_dbg(&priv->rw_client->dev, "ES9080Q initialization complete\n");

	return 0;
}

/**
 * es9080q_hw_params - Hardware parameters configuration (initialization trigger)
 * @substream: PCM substream (playback or capture)
 * @params: Hardware parameters
 * @dai: DAI structure
 *
 * Called by ALSA when audio playback starts with specific sample rate and format.
 * Clock dividers are recalculated for the new sample rate, TDM slot allocation
 * is verified, and the DAC is resynced for the new rate.
 *
 * Return: 0 on success, negative error code on initialization failure
 */
static int es9080q_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	int ret;

	dev_dbg(component->dev, "ES9080Q hw_params: rate=%u\n", rate);

	ret = es9080q_initialize_codec(priv, rate);
	if (ret) {
		dev_err(component->dev,
			"ES9080Q codec initialization failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int es9080q_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "ES9080Q set_fmt: 0x%08x\n", fmt);

	/* ES9080Q uses TDM/DSP mode internally */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		dev_err(component->dev, "Unsupported format\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		priv->is_master = false;
		dev_dbg(component->dev, "ES9080Q: Slave mode\n");
		break;
	case SND_SOC_DAIFMT_CBP_CFP:
		priv->is_master = true;
		dev_dbg(component->dev, "ES9080Q: Master mode\n");
		break;
	default:
		dev_err(component->dev, "Unsupported clock provider mode\n");
		return -EINVAL;
	}

	return 0;
}

static int es9080q_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			      unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);

	if (!freq)
		return -EINVAL;

	dev_dbg(component->dev, "ES9080Q set_sysclk: freq=%u\n", freq);
	priv->sysclk_freq = freq;

	return 0;
}

/**
 * es9080q_set_tdm_slot - Store TDM slot configuration for use at stream start
 * @dai: DAI structure
 * @tx_mask: Bitmask of active TX (playback) slots
 * @rx_mask: Bitmask of active RX (capture) slots — unused (DAC only)
 * @slots: Total number of TDM slots in the frame
 * @slot_width: Width of each slot in bits
 *
 * Stores the TDM slot parameters received from the machine driver. The actual
 * register writes are deferred to es9080q_initialize_codec() because BCLK must
 * be running before TDM registers can be programmed reliably.
 *
 * For the Bela Audio Cape the machine driver passes tx_mask=0x03FC (slots 2-9),
 * placing ES9080Q channels 0-7 into slots 2-9 and leaving slots 0-1 for the
 * TLV320AIC3104 codec.
 *
 * Return: 0 on success, negative error code on invalid configuration
 */
static int es9080q_set_tdm_slot(struct snd_soc_dai *dai,
				unsigned int tx_mask, unsigned int rx_mask,
				int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	int first_slot = -1;
	int active_slots = 0;
	int i;

	dev_dbg(component->dev,
		"ES9080Q set_tdm_slot: tx_mask=0x%04x rx_mask=0x%04x slots=%d width=%d\n",
		tx_mask, rx_mask, slots, slot_width);

	/* Count active TX slots and find the first one */
	for (i = 0; i < slots && i < 32; i++) {
		if (tx_mask & (1 << i)) {
			if (first_slot == -1)
				first_slot = i;
			active_slots++;
		}
	}

	if (rx_mask) {
		dev_err(component->dev, "RX slots not supported on ES9080Q\n");
		return -EINVAL;
	}

	if (slots != 16 || slot_width != 32) {
		dev_err(component->dev,
			"Only 16-slot/32-bit TDM mode supported in v1\n");
		return -EINVAL;
	}

	if (active_slots != ES9080Q_NUM_OUT_CHANNELS) {
		dev_err(component->dev,
			"Need exactly %d TX slots, got %d\n",
			ES9080Q_NUM_OUT_CHANNELS, active_slots);
		return -EINVAL;
	}

	if (first_slot < 0 ||
	    first_slot > (slots - ES9080Q_NUM_OUT_CHANNELS)) {
		dev_err(component->dev,
			"Invalid first slot %d for %d slots\n",
			first_slot, slots);
		return -EINVAL;
	}

	if ((tx_mask & GENMASK(slots - 1, 0)) !=
	    GENMASK(first_slot + ES9080Q_NUM_OUT_CHANNELS - 1, first_slot)) {
		dev_err(component->dev,
			"TX slots must be contiguous (mask=0x%x)\n",
			tx_mask);
		return -EINVAL;
	}

	/*
	 * Store configuration — register writes are deferred until
	 * es9080q_initialize_codec() when BCLK is guaranteed stable.
	 */
	priv->tdm_tx_mask    = tx_mask;
	priv->tdm_slots      = slots;
	priv->tdm_slot_width = slot_width;
	priv->tdm_first_slot = (first_slot == -1) ? 0 : first_slot;

	dev_dbg(component->dev,
		"ES9080Q TDM slots stored: first_slot=%d active=%d\n",
		priv->tdm_first_slot, active_slots);

	return 0;
}

static int es9080q_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(component->dev, "ES9080Q: Audio START\n");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_dbg(component->dev, "ES9080Q: Audio STOP\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * es9080q_mute_stream - Mute/unmute the audio DAC
 * @dai: DAI structure
 * @mute: 1 to mute, 0 to unmute
 * @direction: PCM stream direction
 *
 * Mutes or unmutes the ES9080Q amplifier via AMP_CTRL.
 * Playback streams only — ES9080Q is a DAC (output only).
 *
 * Return: 0 on success, negative error code on failure
 */
static int es9080q_mute_stream(struct snd_soc_dai *dai, int mute,
			       int direction)
{
	struct snd_soc_component *component = dai->component;
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);

	/* ES9080Q is playback-only */
	if (direction != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	/* Skip if codec hasn't been initialized yet */
	if (!priv->codec_initialized)
		return 0;

	if (mute) {
		dev_dbg(component->dev, "ES9080Q: Muting amplifier\n");
		return es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
					ES9080Q_AMP_CTRL_MUTE);
	}

	dev_dbg(component->dev, "ES9080Q: Unmuting amplifier\n");
	return es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
				ES9080Q_AMP_CTRL_UNMUTE);
}

/**
 * es9080q_component_suspend - Suspend codec during system sleep
 * @component: Component structure
 *
 * Mutes the amplifier on system suspend to reduce power consumption.
 *
 * Return: 0 on success, negative error code on failure
 */
static int es9080q_component_suspend(struct snd_soc_component *component)
{
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "ES9080Q: Suspending\n");

	if (priv->codec_initialized) {
		ret = es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
				       ES9080Q_AMP_CTRL_MUTE);
		if (ret)
			dev_warn(component->dev,
				 "Failed to mute during suspend: %d\n", ret);
	}

	return 0;
}

/**
 * es9080q_component_resume - Resume codec after system sleep
 * @component: Component structure
 *
 * Unmutes the amplifier on system resume.
 *
 * Return: 0 on success, negative error code on failure
 */
static int es9080q_component_resume(struct snd_soc_component *component)
{
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "ES9080Q: Resuming\n");

	if (priv->codec_initialized) {
		ret = es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
				       ES9080Q_AMP_CTRL_UNMUTE);
		if (ret)
			dev_warn(component->dev,
				 "Failed to unmute during resume: %d\n", ret);
	}

	return 0;
}

static const struct snd_soc_dai_ops es9080q_dai_ops = {
	.hw_params    = es9080q_hw_params,
	.set_fmt      = es9080q_set_fmt,
	.set_sysclk   = es9080q_set_sysclk,
	.set_tdm_slot = es9080q_set_tdm_slot,
	.trigger      = es9080q_trigger,
	.mute_stream  = es9080q_mute_stream,
};

static struct snd_soc_dai_driver es9080q_dai = {
	.name = "es9080q-hifi",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = ES9080Q_NUM_OUT_CHANNELS,
		.rates        = ES9080Q_RATES,
		.formats      = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es9080q_dai_ops,
};

/* --------------------------------------------------------------------------
 * Volume controls
 * --------------------------------------------------------------------------
 * SOC_SINGLE_EXT is used so that the channel index is encoded in the mc->reg
 * field (first argument after the name string), which get/put retrieve via
 * soc_mixer_control.  This follows the standard kernel pattern and avoids
 * open-coding an info/get/put triplet.
 * -------------------------------------------------------------------------- */

#define ES9080Q_VOL_CTRL(xname, xchannel) \
	SOC_SINGLE_EXT(xname, xchannel, 0, ES9080Q_VOLUME_MAX, 0, \
		       es9080q_volume_get, es9080q_volume_put)

static int es9080q_volume_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_kcontrol_chip(kcontrol);
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int channel = mc->reg;

	if (channel >= ES9080Q_NUM_OUT_CHANNELS)
		return -EINVAL;

	ucontrol->value.integer.value[0] = priv->volume[channel];
	return 0;
}

static int es9080q_volume_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_kcontrol_chip(kcontrol);
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int channel = mc->reg;
	int volume = ucontrol->value.integer.value[0];
	int ret;

	if (channel >= ES9080Q_NUM_OUT_CHANNELS)
		return -EINVAL;

	if (volume < ES9080Q_VOLUME_MIN || volume > ES9080Q_VOLUME_MAX)
		return -EINVAL;

	if (priv->volume[channel] == volume)
		return 0;

	priv->volume[channel] = volume;

	if (priv->codec_initialized) {
		ret = es9080q_rw_write(priv, ES9080Q_REG_VOLUME_BASE + channel,
				       volume);
		if (ret)
			return ret;
	}

	return 1;  /* Indicate value changed */
}

static const struct snd_kcontrol_new es9080q_controls[] = {
	ES9080Q_VOL_CTRL("DAC1 Playback Volume", 0),
	ES9080Q_VOL_CTRL("DAC2 Playback Volume", 1),
	ES9080Q_VOL_CTRL("DAC3 Playback Volume", 2),
	ES9080Q_VOL_CTRL("DAC4 Playback Volume", 3),
	ES9080Q_VOL_CTRL("DAC5 Playback Volume", 4),
	ES9080Q_VOL_CTRL("DAC6 Playback Volume", 5),
	ES9080Q_VOL_CTRL("DAC7 Playback Volume", 6),
	ES9080Q_VOL_CTRL("DAC8 Playback Volume", 7),
};

/* --------------------------------------------------------------------------
 * DAPM
 *
 * The ES9080Q has a single DAC core that drives all 8 output pins.
 * One DAC widget is used (matching the single "Playback" stream) with 8
 * separate OUTPUT pins — DAPM powers the DAC block on stream open and each
 * output pin can be individually connected by the machine driver's routes.
 * -------------------------------------------------------------------------- */
static const struct snd_soc_dapm_widget es9080q_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT1"),
	SND_SOC_DAPM_OUTPUT("OUT2"),
	SND_SOC_DAPM_OUTPUT("OUT3"),
	SND_SOC_DAPM_OUTPUT("OUT4"),
	SND_SOC_DAPM_OUTPUT("OUT5"),
	SND_SOC_DAPM_OUTPUT("OUT6"),
	SND_SOC_DAPM_OUTPUT("OUT7"),
	SND_SOC_DAPM_OUTPUT("OUT8"),
};

static const struct snd_soc_dapm_route es9080q_dapm_routes[] = {
	{ "OUT1", NULL, "DAC" },
	{ "OUT2", NULL, "DAC" },
	{ "OUT3", NULL, "DAC" },
	{ "OUT4", NULL, "DAC" },
	{ "OUT5", NULL, "DAC" },
	{ "OUT6", NULL, "DAC" },
	{ "OUT7", NULL, "DAC" },
	{ "OUT8", NULL, "DAC" },
};

static int es9080q_component_probe(struct snd_soc_component *component)
{
	struct es9080q_priv *priv = snd_soc_component_get_drvdata(component);
	int i;

	dev_dbg(component->dev,
		"ES9080Q component probe — codec init deferred to hw_params\n");

	priv->codec_initialized = false;

	/* Default volumes to 0 dB (register value 0x00) */
	for (i = 0; i < ES9080Q_NUM_OUT_CHANNELS; i++)
		priv->volume[i] = 0;

	/*
	 * Default TDM slot: first_slot=0.  The machine driver should call
	 * set_tdm_slot() to override this before stream start.
	 */
	priv->tdm_first_slot = 0;

	return 0;
}

static const struct snd_soc_component_driver soc_component_dev_es9080q = {
	.probe            = es9080q_component_probe,
	.suspend          = es9080q_component_suspend,
	.resume           = es9080q_component_resume,
	.controls         = es9080q_controls,
	.num_controls     = ARRAY_SIZE(es9080q_controls),
	.dapm_widgets     = es9080q_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es9080q_dapm_widgets),
	.dapm_routes      = es9080q_dapm_routes,
	.num_dapm_routes  = ARRAY_SIZE(es9080q_dapm_routes),
};

/* --------------------------------------------------------------------------
 * Regmap configurations
 * -------------------------------------------------------------------------- */

static bool es9080q_wo_readable(struct device *dev, unsigned int reg)
{
	return false;  /* Write-only registers are not readable */
}

static const struct regmap_config es9080q_wo_regmap_config = {
	.name         = "write-only",
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = 0xFF,
	.readable_reg = es9080q_wo_readable,
	.cache_type   = REGCACHE_NONE,
};

static const struct regmap_config es9080q_rw_regmap_config = {
	.name             = "read-write",
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = 0xFF,
	.reg_defaults     = es9080q_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es9080q_reg_defaults),
	.cache_type       = REGCACHE_RBTREE,
};

/* --------------------------------------------------------------------------
 * I2C probe / remove
 * -------------------------------------------------------------------------- */

static int es9080q_i2c_probe(struct i2c_client *rw_client)
{
	struct device *dev = &rw_client->dev;
	struct es9080q_priv *priv;
	u32 wo_addr;
	int ret;
	int i;

	dev_dbg(dev, "ES9080Q I2C probe\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rw_client = rw_client;
	i2c_set_clientdata(rw_client, priv);

	/* Get write-only I2C address from Device Tree */
	ret = of_property_read_u32(dev->of_node, "write-only-addr", &wo_addr);
	if (ret) {
		dev_err(dev, "Missing write-only-addr property\n");
		return ret;
	}

	if (wo_addr > 0x7f || wo_addr == rw_client->addr) {
		dev_err(dev, "Invalid write-only-addr 0x%x\n", wo_addr);
		return -EINVAL;
	}

	/* Create write-only dummy client on the same adapter */
	priv->wo_client = i2c_new_dummy_device(rw_client->adapter, wo_addr);
	if (IS_ERR(priv->wo_client)) {
		dev_err(dev, "Failed to create WO client\n");
		return PTR_ERR(priv->wo_client);
	}

	/* Initialize regmaps */
	priv->rw_regmap = devm_regmap_init_i2c(priv->rw_client,
					       &es9080q_rw_regmap_config);
	if (IS_ERR(priv->rw_regmap)) {
		ret = PTR_ERR(priv->rw_regmap);
		goto err_wo_device;
	}

	priv->wo_regmap = devm_regmap_init_i2c(priv->wo_client,
					       &es9080q_wo_regmap_config);
	if (IS_ERR(priv->wo_regmap)) {
		ret = PTR_ERR(priv->wo_regmap);
		goto err_wo_device;
	}

	/* Get optional reset GPIO (active-low) */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		ret = PTR_ERR(priv->reset_gpio);
		goto err_wo_device;
	}

	/* Hardware reset */
	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0); /* Assert */
		udelay(ES9080Q_RESET_ASSERT_US);
		gpiod_set_value_cansleep(priv->reset_gpio, 1); /* Release */
		udelay(ES9080Q_RESET_RELEASE_US);
	}

	/* Write PLL/GPIO initialization sequence via write-only address */
	for (i = 0; i < ARRAY_SIZE(es9080q_init_seq); i++) {
		ret = es9080q_wo_write(priv, es9080q_init_seq[i].reg,
				       es9080q_init_seq[i].def);
		if (ret) {
			dev_warn(dev, "Init seq step %d failed: %d\n", i, ret);
			continue;
		}
		udelay(50);
	}

	/* Default MCLK frequency; machine driver updates via set_sysclk */
	priv->sysclk_freq = 24576000;

	/* Register ASoC component */
	ret = devm_snd_soc_register_component(dev, &soc_component_dev_es9080q,
					      &es9080q_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register component: %d\n", ret);
		goto err_wo_device;
	}

	dev_info(dev, "ES9080Q codec registered successfully (8 channels)\n");

	return 0;

err_wo_device:
	i2c_unregister_device(priv->wo_client);
	return ret;
}

static void es9080q_i2c_remove(struct i2c_client *client)
{
	struct es9080q_priv *priv = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "ES9080Q I2C remove\n");

	/*
	 * Shutdown order:
	 * 1. Mute the amplifier gracefully (avoids pops)
	 * 2. Assert reset GPIO to power down the chip
	 * 3. Unregister the non-devm wo_client
	 *    (devm_ handles: regmaps, component, kzalloc, reset_gpio)
	 */
	if (priv->codec_initialized)
		es9080q_rw_write(priv, ES9080Q_REG_AMP_CTRL,
				 ES9080Q_AMP_CTRL_MUTE);

	if (priv->reset_gpio)
		gpiod_set_value_cansleep(priv->reset_gpio, 0);

	if (priv->wo_client)
		i2c_unregister_device(priv->wo_client);
}

/* Device Tree match table */
static const struct of_device_id es9080q_of_match[] = {
	{ .compatible = "ess,es9080q" },
	{ }
};
MODULE_DEVICE_TABLE(of, es9080q_of_match);

/* I2C device ID table */
static const struct i2c_device_id es9080q_i2c_id[] = {
	{ "es9080q", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es9080q_i2c_id);

static struct i2c_driver es9080q_i2c_driver = {
	.driver = {
		.name           = "es9080q",
		.of_match_table = es9080q_of_match,
	},
	.probe    = es9080q_i2c_probe,
	.remove   = es9080q_i2c_remove,
	.id_table = es9080q_i2c_id,
};
module_i2c_driver(es9080q_i2c_driver);

MODULE_DESCRIPTION("ASoC ES9080Q codec driver");
MODULE_AUTHOR("JianDe jiande2020@gmail.com");
MODULE_AUTHOR("Piyush Patle piyushpatle1228@gmail.com");
MODULE_AUTHOR("Giulio Moro");
MODULE_LICENSE("GPL");
