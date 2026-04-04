// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESS ES9080Q 8-channel 32-bit DAC — ASoC codec driver
 *
 * The ES9080Q is an I2C-controlled, TDM-input 8-channel HiFi DAC.
 * It receives audio via up to two TDM serial data lines (DATA1, DATA2),
 * each carrying 4 channels at 32-bit/slot in TDM8 format.
 *
 * Hardware connections on Bela Gem Multi (PocketBeagle2):
 *   I2C:      main_i2c1 (bus 2 in Linux)
 *   TDM A:    McASP2_AXR3 (P1.19) → ES9080Q DATA  (ch 0–7)
 *   BCLK:     P2.19 (McASP2_ACLKX, sourced from TLV320AIC3106 PLL)
 *   WCLK:     P2.10 (McASP2_AFSX,  sourced from TLV320AIC3106)
 *
 * The ES9080Q has two I2C interfaces:
 *   Read/Write (primary):  registers 0–164 (0x00–0xA4) R/W,
 *                          registers 224–255 (0xE0–0xFF) read-only
 *   Write-only (secondary): registers 192–203 (0xC0–0xCB) write-only
 *                           Used for reset and PLL configuration.
 *   The write-only address is always primary_address + 4
 *   (e.g. primary=0x48, write-only=0x4C).
 *
 * Register map verified against:
 *   - bela-org-info/Bela/core/Es9080_Codec.cpp (Bela userspace driver)
 *   - ES9080Q datasheet Rev 0.4
 *
 * Author: Piyush Patle <piyushpatle1228@gmail.com>
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* ------------------------------------------------------------------ */
/* Register map — verified against Es9080_Codec.cpp + datasheet       */
/* ------------------------------------------------------------------ */

/*
 * Registers 0–164 (0x00–0xA4): Read/Write — accessed via primary I2C addr
 * Registers 192–203 (0xC0–0xCB): Write-only — accessed via secondary I2C addr
 * Registers 224–255 (0xE0–0xFF): Read-only — accessed via primary I2C addr
 */

/* Core control (primary address, R/W) */
#define ES9080Q_REG_AMP_CTRL		0x00	/* Amplifier control: 0x02 = turn on */
#define ES9080Q_REG_INTERP_MOD_CLK	0x01	/* Interpolation & modulator clocks */
#define ES9080Q_REG_TDM_DECODER		0x02	/* TDM decoder enable: 0x01 = on */
#define ES9080Q_REG_DAC_CONFIG		0x03	/* SELECT_IDAC_NUM: DAC clock divider */
#define ES9080Q_REG_MASTER_CLK		0x04	/* SELECT_MENC_NUM: master clock div */
#define ES9080Q_REG_ANALOG_EN		0x05	/* Analog section enable per channel */
#define ES9080Q_REG_CP_CLK_DIV		0x06	/* Charge pump clock divider */
#define ES9080Q_REG_ANALOG_DELAY	0x07	/* Analog delay sequence for quiet pop */

#define ES9080Q_REG_PLL_LOCK		0x33	/* reg 51: force PLL lock signal */

/* TDM configuration */
#define ES9080Q_REG_INPUT_CONFIG	0x4D	/* reg 77: master mode + TDM select */
#define ES9080Q_REG_MASTER_MODE		0x4E	/* reg 78: master BCK/WS config */
#define ES9080Q_REG_TDM_CONFIG1		0x4F	/* reg 79: WS scale, slot count */
#define ES9080Q_REG_TDM_CONFIG2		0x50	/* reg 80: LJ mode, valid edge */
#define ES9080Q_REG_TDM_CONFIG3		0x51	/* reg 81: bit width per slot */
#define ES9080Q_REG_BCK_WS_MON		0x52	/* reg 82: BCK/WS monitor */
#define ES9080Q_REG_TDM_VALID_PULSE	0x53	/* reg 83: valid pulse position */

/* TDM channel-to-slot mapping: regs 84–91 (0x54–0x5B) */
#define ES9080Q_REG_TDM_CH1_CFG	0x54	/* reg 84 */
#define ES9080Q_REG_TDM_CH2_CFG	0x55
#define ES9080Q_REG_TDM_CH3_CFG	0x56
#define ES9080Q_REG_TDM_CH4_CFG	0x57
#define ES9080Q_REG_TDM_CH5_CFG	0x58
#define ES9080Q_REG_TDM_CH6_CFG	0x59
#define ES9080Q_REG_TDM_CH7_CFG	0x5A
#define ES9080Q_REG_TDM_CH8_CFG	0x5B

#define ES9080Q_REG_DAC_RESYNC		0x5C	/* reg 92: DAC clock resync */

/* Per-channel volume: regs 94–101 (0x5E–0x65), 0.5 dB attenuation steps */
#define ES9080Q_REG_VOL_CH1		0x5E	/* reg 94 */
#define ES9080Q_REG_VOL_CH2		0x5F
#define ES9080Q_REG_VOL_CH3		0x60
#define ES9080Q_REG_VOL_CH4		0x61
#define ES9080Q_REG_VOL_CH5		0x62
#define ES9080Q_REG_VOL_CH6		0x63
#define ES9080Q_REG_VOL_CH7		0x64
#define ES9080Q_REG_VOL_CH8		0x65

#define ES9080Q_REG_VOL_CTRL		0x69	/* reg 105: volume & mono control */

/* Filter and dither */
#define ES9080Q_REG_FILTER_CFG		0x6C	/* reg 108 */
#define ES9080Q_REG_DITHER_CFG		0x6D	/* reg 109 */

/* THD compensation */
#define ES9080Q_REG_THD_C2_ODD		0x6F	/* reg 111: C2 coeff CH1/3/5/7 */
#define ES9080Q_REG_THD_C2H_ODD	0x70	/* reg 112 */
#define ES9080Q_REG_THD_C3_ODD		0x71	/* reg 113: C3 coeff CH1/3/5/7 */
#define ES9080Q_REG_THD_C2_EVEN	0x73	/* reg 115: C2 coeff CH2/4/6/8 */
#define ES9080Q_REG_THD_C2H_EVEN	0x74	/* reg 116 */
#define ES9080Q_REG_THD_C3_EVEN	0x75	/* reg 117: C3 coeff CH2/4/6/8 */

#define ES9080Q_REG_AUTOMUTE		0x77	/* reg 119 */

/* Noise shaping modulator */
#define ES9080Q_REG_NSMOD_PHASE		0x80	/* reg 128 */
#define ES9080Q_REG_NSMOD_TYPE		0x81	/* reg 129 */
#define ES9080Q_REG_NSMOD_DITH_12	0x83	/* reg 131: dither CH1/2 */
#define ES9080Q_REG_NSMOD_DITH_34	0x84	/* reg 132: dither CH3/4 */
#define ES9080Q_REG_NSMOD_DITH_56	0x85	/* reg 133: dither CH5/6 */
#define ES9080Q_REG_NSMOD_DITH_78	0x86	/* reg 134: dither CH7/8 */

#define ES9080Q_REG_DRE_CTRL		0x88	/* reg 136 */
#define ES9080Q_REG_GAIN_18DB		0x9A	/* reg 154: per-channel 18dB boost */

/* Write-only registers (secondary I2C address = primary + 4) */
#define ES9080Q_REG_RESET_PLL1		0xC0	/* reg 192: soft reset + PLL reg 1 */
#define ES9080Q_REG_PLL_CONFIG		0xC1	/* reg 193: GPIO1 mode, PLL bypass */
#define ES9080Q_REG_PLL_PARAMS		0xCA	/* reg 202: PLL parameters */

/* Bit definitions */
#define ES9080Q_AMP_ON			0x02
#define ES9080Q_AMP_OFF			0x00
#define ES9080Q_ALL_CH_EN		0xFF
#define ES9080Q_TDM_DECODER_EN		0x01
#define ES9080Q_FORCE_PLL_LOCK		0x80
#define ES9080Q_SOFT_RESET		0xC0	/* AO_SOFT_RESET | PLL_SOFT_RESET */

/* ES9080Q_REG_INPUT_CONFIG bits */
#define ES9080Q_MASTER_MODE_EN		BIT(4)
#define ES9080Q_INPUT_SEL_TDM		(0 << 2)

/* ES9080Q_REG_TDM_CONFIG2 bits */
#define ES9080Q_TDM_LJ_MODE		BIT(7)
#define ES9080Q_TDM_VALID_PULSE_8	0x08

/* ES9080Q_REG_TDM_CONFIG3 bit width encoding */
#define ES9080Q_TDM_32BIT		(0 << 6)
#define ES9080Q_TDM_24BIT		(1 << 6)
#define ES9080Q_TDM_16BIT		(2 << 6)

/* ES9080Q_REG_VOL_CTRL bits */
#define ES9080Q_FORCE_VOLUME		BIT(6)

/* ------------------------------------------------------------------ */
/* Private driver state                                                */
/* ------------------------------------------------------------------ */

struct es9080q_priv {
	struct i2c_client	*i2c;		/* primary (R/W) address */
	struct i2c_client	*i2c_wo;	/* write-only address */
	struct gpio_desc	*reset_gpio;
	unsigned int		 sysclk;
	unsigned int		 fmt;
};

/* ------------------------------------------------------------------ */
/* Low-level I2C register access                                       */
/*                                                                     */
/* The ES9080Q uses two I2C addresses:                                 */
/*   primary (R/W): regs 0–164 and 224–255                            */
/*   secondary (write-only): regs 192–203                             */
/* ------------------------------------------------------------------ */

static struct i2c_client *es9080q_client_for_reg(struct es9080q_priv *es,
						  u8 reg, bool write)
{
	/* Write-only registers 192–203 go to secondary address */
	if (reg >= 192 && reg <= 203 && write)
		return es->i2c_wo ? es->i2c_wo : es->i2c;

	/* Everything else goes to primary address */
	return es->i2c;
}

static int es9080q_write(struct es9080q_priv *es, u8 reg, u8 val)
{
	struct i2c_client *client = es9080q_client_for_reg(es, reg, true);
	u8 buf[2] = { reg, val };
	int ret;

	ret = i2c_master_send(client, buf, 2);
	if (ret < 0)
		return ret;
	return ret == 2 ? 0 : -EIO;
}

static int es9080q_read(struct es9080q_priv *es, u8 reg, u8 *val)
{
	struct i2c_client *client = es9080q_client_for_reg(es, reg, false);
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
		return ret;

	return ret == 1 ? 0 : -EIO;
}

/* snd_soc_component read/write callbacks used by DAPM and controls */
static int es9080q_component_write(struct snd_soc_component *component,
				   unsigned int reg, unsigned int val)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(component);

	return es9080q_write(es, (u8)reg, (u8)val);
}

static unsigned int es9080q_component_read(struct snd_soc_component *component,
					   unsigned int reg)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(component);
	u8 val;
	int ret;

	ret = es9080q_read(es, (u8)reg, &val);
	if (ret < 0)
		return (unsigned int)ret;
	return val;
}

/* ------------------------------------------------------------------ */
/* Hardware initialisation sequence                                    */
/*                                                                     */
/* Derived from Es9080_Codec.cpp startAudio() in bela-org-info.       */
/* The ES9080Q on Bela Gem Multi is always TDM slave — clocks come    */
/* from the TLV320AIC3106.                                            */
/* ------------------------------------------------------------------ */

static int es9080q_hw_init(struct es9080q_priv *es)
{
	int ret;
	int i;

	/* Assert/deassert hardware reset if GPIO is available */
	if (es->reset_gpio) {
		gpiod_set_value_cansleep(es->reset_gpio, 0);
		msleep(10);
		gpiod_set_value_cansleep(es->reset_gpio, 1);
		msleep(10);
	}

	/*
	 * Step 1: PLL/GPIO registers (write-only address)
	 * Set GPIO1 (MCLK) pad to input mode, invert CLKHV for better DNR
	 */
	ret = es9080q_write(es, ES9080Q_REG_RESET_PLL1, 0x03);
	if (ret)
		return ret;

	/* PLL bypass, remove DVDD shunt, set PLL input to MCLK */
	ret = es9080q_write(es, ES9080Q_REG_PLL_CONFIG, 0xC3);
	if (ret)
		return ret;

	/* PLL parameters */
	ret = es9080q_write(es, ES9080Q_REG_PLL_PARAMS, 0x40);
	if (ret)
		return ret;

	/*
	 * Step 2: Core registers (primary address)
	 * Enable interpolation and modulator clocks for all 8 channels
	 */
	ret = es9080q_write(es, ES9080Q_REG_INTERP_MOD_CLK, ES9080Q_ALL_CH_EN);
	if (ret)
		return ret;

	/* Enable TDM decoder */
	ret = es9080q_write(es, ES9080Q_REG_TDM_DECODER, ES9080Q_TDM_DECODER_EN);
	if (ret)
		return ret;

	/*
	 * DAC config: SELECT_IDAC_NUM
	 * For 256-bit frame with MCLK = 256*Fs: IDAC = (256/128) - 1 = 1
	 */
	ret = es9080q_write(es, ES9080Q_REG_DAC_CONFIG, 0x01);
	if (ret)
		return ret;

	/*
	 * Master clock config: SELECT_MENC_NUM
	 * Slave mode: this still sets the internal decoder reference.
	 * MCLK_OVER_BCK = 2, MASTER_BCK_DIV1 = 1
	 * divide_value_menc = 2 / 1 = 2, SELECT_MENC_NUM = 2 - 1 = 1
	 */
	ret = es9080q_write(es, ES9080Q_REG_MASTER_CLK, 0x01);
	if (ret)
		return ret;

	/*
	 * Charge pump clock divider
	 * CP clock = SYS_CLK / ((CP_CLK_DIV + 1) * 2)
	 * Target: 500 kHz – 1 MHz. For 12 MHz MCLK:
	 * CP_CLK_DIV = 5 → 12M / (6*2) = 1 MHz
	 */
	ret = es9080q_write(es, ES9080Q_REG_CP_CLK_DIV, 0x05);
	if (ret)
		return ret;

	/* Enable all 8 channels analog section */
	ret = es9080q_write(es, ES9080Q_REG_ANALOG_EN, ES9080Q_ALL_CH_EN);
	if (ret)
		return ret;

	/* Analog delay sequence for quiet pop */
	ret = es9080q_write(es, ES9080Q_REG_ANALOG_DELAY, 0xBB);
	if (ret)
		return ret;

	/* Force PLL lock signal (PLL is bypassed) */
	ret = es9080q_write(es, ES9080Q_REG_PLL_LOCK, ES9080Q_FORCE_PLL_LOCK);
	if (ret)
		return ret;

	/*
	 * Step 3: TDM configuration
	 * Slave mode (no MASTER_MODE_EN), TDM input selected
	 */
	ret = es9080q_write(es, ES9080Q_REG_INPUT_CONFIG,
			    ES9080Q_INPUT_SEL_TDM);
	if (ret)
		return ret;

	/*
	 * TDM CONFIG1: 8 TDM slots per frame (TDM_CH_NUM = 7)
	 * MASTER_WS_SCALE = 0 (not in master mode, but set for consistency)
	 */
	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG1, 0x07);
	if (ret)
		return ret;

	/* TDM CONFIG2: Left justified, negative valid edge, 8-slot pulse */
	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG2,
			    ES9080Q_TDM_LJ_MODE | ES9080Q_TDM_VALID_PULSE_8);
	if (ret)
		return ret;

	/* TDM CONFIG3: 32-bit slot width, no daisy chain */
	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG3, ES9080Q_TDM_32BIT);
	if (ret)
		return ret;

	/* Disable BCK/WS monitor */
	ret = es9080q_write(es, ES9080Q_REG_BCK_WS_MON, 0x00);
	if (ret)
		return ret;

	/* TDM valid pulse position = 0 */
	ret = es9080q_write(es, ES9080Q_REG_TDM_VALID_PULSE, 0x00);
	if (ret)
		return ret;

	/*
	 * TDM channel-to-slot mapping: CH1→slot0, CH2→slot1, ..., CH8→slot7
	 * All channels on DATA line 1 (TDM_CHx_LINE_SEL = 0)
	 */
	for (i = 0; i < 8; i++) {
		ret = es9080q_write(es, ES9080Q_REG_TDM_CH1_CFG + i, i);
		if (ret)
			return ret;
	}

	/*
	 * Step 4: Filter and dither configuration
	 */
	/* Minimum phase slow roll-off filter, no de-emphasis */
	ret = es9080q_write(es, ES9080Q_REG_FILTER_CFG, 0x46);
	if (ret)
		return ret;

	/* IIR filter dither for best low-level linearity */
	ret = es9080q_write(es, ES9080Q_REG_DITHER_CFG, 0xE4);
	if (ret)
		return ret;

	/* THD compensation coefficients */
	ret = es9080q_write(es, ES9080Q_REG_THD_C2_ODD, 0x68);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2H_ODD, 0x01);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C3_ODD, 0x8D);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2_EVEN, 0x68);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2H_EVEN, 0x01);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C3_EVEN, 0x8D);
	if (ret)
		return ret;

	/* Disable automute */
	ret = es9080q_write(es, ES9080Q_REG_AUTOMUTE, 0x00);
	if (ret)
		return ret;

	/* NSMOD dither for best performance */
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_PHASE, 0xCC);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_TYPE, 0x54);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_DITH_12, 0x44);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_DITH_34, 0x44);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_DITH_56, 0x44);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_DITH_78, 0x44);
	if (ret)
		return ret;

	/* DRE: volume-dependent gain off for now */
	ret = es9080q_write(es, ES9080Q_REG_DRE_CTRL, 0x00);
	if (ret)
		return ret;

	/*
	 * Step 5: DAC clock resync — 3 sequential writes required
	 * Aligns all clocks in the DAC core for best analog performance
	 */
	ret = es9080q_write(es, ES9080Q_REG_DAC_RESYNC, 0x10);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_DAC_RESYNC, 0x0F);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_DAC_RESYNC, 0x00);
	if (ret)
		return ret;

	/*
	 * Step 6: Turn on amplifier
	 * This runs a state machine to gracefully power up the DACs.
	 */
	ret = es9080q_write(es, ES9080Q_REG_AMP_CTRL, ES9080Q_AMP_ON);
	if (ret)
		return ret;

	/*
	 * Step 7: Volume control — force immediate volume updates,
	 * set all channels to 0 dB
	 */
	ret = es9080q_write(es, ES9080Q_REG_VOL_CTRL, ES9080Q_FORCE_VOLUME);
	if (ret)
		return ret;

	for (i = 0; i < 8; i++) {
		ret = es9080q_write(es, ES9080Q_REG_VOL_CH1 + i, 0x00);
		if (ret)
			return ret;
	}

	dev_info(&es->i2c->dev, "ES9080Q initialised (8ch TDM slave)\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Component probe                                                     */
/* ------------------------------------------------------------------ */

static int es9080q_probe(struct snd_soc_component *component)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(component);

	/*
	 * Verify the device is present by reading a known register.
	 * Reg 0 (AMP_CTRL) should be readable at the primary address.
	 * The ES9080Q has no dedicated chip ID register.
	 */
	u8 val;
	int ret;

	ret = es9080q_read(es, ES9080Q_REG_AMP_CTRL, &val);
	if (ret) {
		dev_err(component->dev,
			"failed to read ES9080Q at I2C 0x%02x: %d\n",
			es->i2c->addr, ret);
		return ret;
	}
	dev_info(component->dev,
		 "ES9080Q detected at I2C 0x%02x (R/W) + 0x%02x (W/O)\n",
		 es->i2c->addr,
		 es->i2c_wo ? es->i2c_wo->addr : 0);

	return es9080q_hw_init(es);
}

/* ------------------------------------------------------------------ */
/* DAI operations                                                      */
/* ------------------------------------------------------------------ */

static int es9080q_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	/*
	 * ES9080Q locks to BCLK/WCLK automatically in slave mode.
	 * The TDM configuration is set statically in es9080q_hw_init().
	 */
	dev_dbg(dai->dev, "hw_params: rate=%u channels=%u\n",
		params_rate(params), params_channels(params));
	return 0;
}

static int es9080q_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(dai->component);

	es->fmt = fmt;

	/*
	 * ES9080Q on Bela is always TDM slave — BCLK and WCLK come from
	 * TLV320AIC3106. Accept DSP_B (TDM with 1-bit offset) which is
	 * what the Bela hardware uses.
	 */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		dev_err(dai->dev, "unsupported DAI format 0x%x\n", fmt);
		return -EINVAL;
	}

	return 0;
}

static int es9080q_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			      unsigned int freq, int dir)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(dai->component);

	/*
	 * ES9080Q derives timing from BCLK/WCLK directly in slave mode.
	 * Store for reference only.
	 */
	es->sysclk = freq;
	return 0;
}

static int es9080q_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(dai->component);

	/*
	 * Mute/unmute by controlling the amplifier.
	 * AMP_CTRL reg 0: 0x02 = on, 0x00 = off.
	 */
	return es9080q_write(es, ES9080Q_REG_AMP_CTRL,
			     mute ? ES9080Q_AMP_OFF : ES9080Q_AMP_ON);
}

static const struct snd_soc_dai_ops es9080q_dai_ops = {
	.hw_params	 = es9080q_hw_params,
	.set_fmt	 = es9080q_set_fmt,
	.set_sysclk	 = es9080q_set_sysclk,
	.mute_stream	 = es9080q_mute_stream,
	.no_capture_mute = 1,
};

/* ------------------------------------------------------------------ */
/* DAI driver — playback only (ES9080Q is a DAC, no capture path)     */
/* ------------------------------------------------------------------ */

static struct snd_soc_dai_driver es9080q_dai = {
	.name = "es9080q-hifi",
	.playback = {
		.stream_name  = "ES9080Q Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates        = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000  | SNDRV_PCM_RATE_192000,
		.formats      = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es9080q_dai_ops,
};

/* ------------------------------------------------------------------ */
/* DAPM widgets, routes, and mixer controls                            */
/* ------------------------------------------------------------------ */

/*
 * Volume TLV: 0 dB to −127.5 dB in 0.5 dB steps.
 * Register value = attenuation in half-dB units (0 = 0 dB, 255 = −127.5 dB).
 */
static const DECLARE_TLV_DB_SCALE(es9080q_vol_tlv, -12750, 50, 0);

static const struct snd_kcontrol_new es9080q_controls[] = {
	SOC_SINGLE_TLV("Ch1 Volume", ES9080Q_REG_VOL_CH1, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch2 Volume", ES9080Q_REG_VOL_CH2, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch3 Volume", ES9080Q_REG_VOL_CH3, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch4 Volume", ES9080Q_REG_VOL_CH4, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch5 Volume", ES9080Q_REG_VOL_CH5, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch6 Volume", ES9080Q_REG_VOL_CH6, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch7 Volume", ES9080Q_REG_VOL_CH7, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch8 Volume", ES9080Q_REG_VOL_CH8, 0, 255, 1, es9080q_vol_tlv),
};

static const struct snd_soc_dapm_widget es9080q_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC CH1-2", "ES9080Q Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC CH3-4", "ES9080Q Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC CH5-6", "ES9080Q Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC CH7-8", "ES9080Q Playback", SND_SOC_NOPM, 0, 0),
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
	{ "OUT1", NULL, "DAC CH1-2" },
	{ "OUT2", NULL, "DAC CH1-2" },
	{ "OUT3", NULL, "DAC CH3-4" },
	{ "OUT4", NULL, "DAC CH3-4" },
	{ "OUT5", NULL, "DAC CH5-6" },
	{ "OUT6", NULL, "DAC CH5-6" },
	{ "OUT7", NULL, "DAC CH7-8" },
	{ "OUT8", NULL, "DAC CH7-8" },
};

/* ------------------------------------------------------------------ */
/* Component driver struct                                             */
/* ------------------------------------------------------------------ */

static const struct snd_soc_component_driver es9080q_component_driver = {
	.probe			= es9080q_probe,
	.read			= es9080q_component_read,
	.write			= es9080q_component_write,
	.controls		= es9080q_controls,
	.num_controls		= ARRAY_SIZE(es9080q_controls),
	.dapm_widgets		= es9080q_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es9080q_dapm_widgets),
	.dapm_routes		= es9080q_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es9080q_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

/* ------------------------------------------------------------------ */
/* I2C driver                                                          */
/* ------------------------------------------------------------------ */

static int es9080q_i2c_probe(struct i2c_client *i2c)
{
	struct es9080q_priv *es;
	u32 wo_addr;

	es = devm_kzalloc(&i2c->dev, sizeof(*es), GFP_KERNEL);
	if (!es)
		return -ENOMEM;

	es->i2c = i2c;
	i2c_set_clientdata(i2c, es);

	/*
	 * Set up the write-only I2C client for PLL/reset registers.
	 * The DT property "ess,write-only-addr" specifies the secondary
	 * I2C address (typically primary + 4, e.g. 0x48 → 0x4C).
	 */
	if (!of_property_read_u32(i2c->dev.of_node, "ess,write-only-addr",
				  &wo_addr)) {
		es->i2c_wo = devm_i2c_new_dummy_device(&i2c->dev,
							i2c->adapter,
							wo_addr);
		if (IS_ERR(es->i2c_wo)) {
			return dev_err_probe(&i2c->dev, PTR_ERR(es->i2c_wo),
					     "failed to create write-only I2C client at 0x%02x\n",
					     wo_addr);
		}
		dev_info(&i2c->dev,
			 "ES9080Q: primary 0x%02x, write-only 0x%02x\n",
			 i2c->addr, wo_addr);
	} else {
		dev_warn(&i2c->dev,
			 "ES9080Q: no write-only addr, PLL config unavailable\n");
	}

	/*
	 * CHIP_EN / reset GPIO — optional. If the schematic does not wire
	 * a GPIO to CHIP_EN, devm_gpiod_get_optional() returns NULL and
	 * the driver proceeds without hardware reset.
	 */
	es->reset_gpio = devm_gpiod_get_optional(&i2c->dev, "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(es->reset_gpio))
		return dev_err_probe(&i2c->dev, PTR_ERR(es->reset_gpio),
				     "failed to get reset GPIO\n");

	return devm_snd_soc_register_component(&i2c->dev,
					       &es9080q_component_driver,
					       &es9080q_dai, 1);
}

static const struct i2c_device_id es9080q_i2c_id[] = {
	{ "es9080q", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es9080q_i2c_id);

static const struct of_device_id es9080q_of_match[] = {
	{ .compatible = "ess,es9080q" },
	{ }
};
MODULE_DEVICE_TABLE(of, es9080q_of_match);

static struct i2c_driver es9080q_i2c_driver = {
	.driver = {
		.name		= "es9080q",
		.of_match_table	= es9080q_of_match,
	},
	.probe		= es9080q_i2c_probe,
	.id_table	= es9080q_i2c_id,
};
module_i2c_driver(es9080q_i2c_driver);

MODULE_DESCRIPTION("ESS ES9080Q 8-channel DAC ASoC driver");
MODULE_AUTHOR("Piyush Patle <piyushpatle1228@gmail.com>");
MODULE_LICENSE("GPL");
