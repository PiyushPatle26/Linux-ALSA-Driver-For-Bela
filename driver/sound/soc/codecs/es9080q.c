// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESS ES9080Q 8-channel 32-bit DAC — ASoC codec driver
 *
 * The ES9080Q is an I2C-controlled, TDM-input 8-channel HiFi DAC.
 * It supports up to two TDM serial data lines, but on Bela Gem Multi all
 * 8 channels are carried on a SINGLE data line (DATA line 1, slots 0-7,
 * 32-bit/slot, TDM8). This matches Bela's shipping userspace driver
 * (Es9080_Codec.cpp), which sets every channel's TDM_CHx_LINE_SEL to
 * "receive from data line 1". es9080q_hw_init() programs the same mapping.
 *
 * Hardware connections on Bela Gem Multi (PocketBeagle2):
 *   I2C:      main_i2c1 (Linux bus number varies by config -- check i2cdetect -l)
 *   TDM DATA: McASP2_AXR6 (P1.04) -> ES9080Q DATA line 1 (ch 1-8, slots 0-7)
 *   BCLK:     P2.19 (McASP2_ACLKX, sourced from TLV320AIC3106 PLL)
 *   WCLK:     P2.10 (McASP2_AFSX,  sourced from TLV320AIC3106)
 *
 * The ES9080Q has two I2C interfaces:
 *   Read/Write (primary):  registers 0-164 (0x00-0xA4) R/W,
 *                          registers 224-255 (0xE0-0xFF) read-only
 *   Write-only (secondary): registers 192-203 (0xC0-0xCB) write-only
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

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* Register map — verified against Es9080_Codec.cpp + datasheet */

/*
 * Registers 0-164 (0x00-0xA4): Read/Write via primary I2C addr
 * Registers 192-203 (0xC0-0xCB): Write-only via secondary I2C addr
 * Registers 224-255 (0xE0-0xFF): Read-only via primary I2C addr
 */

/* Core control (primary address, R/W) */
#define ES9080Q_REG_AMP_CTRL		0x00
#define ES9080Q_REG_INTERP_MOD_CLK	0x01
#define ES9080Q_REG_TDM_DECODER		0x02
#define ES9080Q_REG_DAC_CONFIG		0x03
#define ES9080Q_REG_MASTER_CLK		0x04
#define ES9080Q_REG_ANALOG_EN		0x05
#define ES9080Q_REG_CP_CLK_DIV		0x06
#define ES9080Q_REG_ANALOG_DELAY	0x07

#define ES9080Q_REG_PLL_LOCK		0x33

/* TDM configuration */
#define ES9080Q_REG_INPUT_CONFIG	0x4D
#define ES9080Q_REG_MASTER_MODE		0x4E
#define ES9080Q_REG_TDM_CONFIG1		0x4F
#define ES9080Q_REG_TDM_CONFIG2		0x50
#define ES9080Q_REG_TDM_CONFIG3		0x51
#define ES9080Q_REG_BCK_WS_MON		0x52
#define ES9080Q_REG_TDM_VALID_PULSE	0x53

/* TDM channel-to-slot mapping: regs 84-91 (0x54-0x5B) */
#define ES9080Q_REG_TDM_CH1_CFG		0x54

#define ES9080Q_REG_DAC_RESYNC		0x5C

/* Per-channel volume: regs 94-101 (0x5E-0x65), 0.5 dB steps */
#define ES9080Q_REG_VOL_CH1		0x5E
#define ES9080Q_REG_VOL_CH2		0x5F
#define ES9080Q_REG_VOL_CH3		0x60
#define ES9080Q_REG_VOL_CH4		0x61
#define ES9080Q_REG_VOL_CH5		0x62
#define ES9080Q_REG_VOL_CH6		0x63
#define ES9080Q_REG_VOL_CH7		0x64
#define ES9080Q_REG_VOL_CH8		0x65

#define ES9080Q_REG_VOL_CTRL		0x69

/* Filter and dither */
#define ES9080Q_REG_FILTER_CFG		0x6C
#define ES9080Q_REG_DITHER_CFG		0x6D

/* THD compensation */
#define ES9080Q_REG_THD_C2_ODD		0x6F
#define ES9080Q_REG_THD_C2H_ODD		0x70
#define ES9080Q_REG_THD_C3_ODD		0x71
#define ES9080Q_REG_THD_C2_EVEN		0x73
#define ES9080Q_REG_THD_C2H_EVEN	0x74
#define ES9080Q_REG_THD_C3_EVEN		0x75

#define ES9080Q_REG_AUTOMUTE		0x77

/* Noise shaping modulator */
#define ES9080Q_REG_NSMOD_PHASE		0x80
#define ES9080Q_REG_NSMOD_TYPE		0x81
#define ES9080Q_REG_NSMOD_DITH_12	0x83
#define ES9080Q_REG_NSMOD_DITH_34	0x84
#define ES9080Q_REG_NSMOD_DITH_56	0x85
#define ES9080Q_REG_NSMOD_DITH_78	0x86

#define ES9080Q_REG_DRE_CTRL		0x88

/* Write-only registers (secondary I2C address = primary + 4) */
#define ES9080Q_REG_RESET_PLL1		0xC0
#define ES9080Q_REG_PLL_CONFIG		0xC1
#define ES9080Q_REG_PLL_PARAMS		0xCA

/* Register range limits (register map: 0x00-0xA4 R/W, 0xC0-0xCB write-only
 * via the secondary address, 0xE0-0xFF read-only status)
 */
#define ES9080Q_RW_REG_MAX		0xA4
#define ES9080Q_WO_REG_MIN		0xC0
#define ES9080Q_WO_REG_MAX		0xCB
#define ES9080Q_RO_REG_MIN		0xE0
#define ES9080Q_MAX_REG			0xFF

/* Bit definitions */
#define ES9080Q_AMP_ON			0x02
#define ES9080Q_ALL_CH_EN		0xFF
#define ES9080Q_TDM_DECODER_EN		0x01
#define ES9080Q_FORCE_PLL_LOCK		0x80
#define ES9080Q_INPUT_SEL_TDM		(0 << 2)

/* TDM CONFIG1 (0x4F): [7] TDM_RESYNC, [6:4] MASTER_WS_SCALE, [3:0] TDM_CH_NUM */
#define ES9080Q_TDM_WS_SCALE_DIV2	BIT(4)
#define ES9080Q_TDM_CH_NUM(slots)	((slots) - 1)

/* TDM CONFIG2 (0x50): [7] TDM_LJ_MODE, [6] TDM_VALID_EDGE, [5:0] TDM_VALID_PULSE_LEN */
#define ES9080Q_TDM_LJ_MODE		BIT(7)
#define ES9080Q_TDM_VALID_EDGE_NEG	(0 << 6)
#define ES9080Q_TDM_VALID_PULSE_LEN_8	0x08

/* TDM CONFIG3 (0x51): [7:6] TDM_BIT_WIDTH (0 = 32-bit slots) */
#define ES9080Q_TDM_32BIT		(0 << 6)

#define ES9080Q_FORCE_VOLUME		BIT(6)

/*
 * RESET & PLL REGISTER1 (0xC0, write-only port):
 * [7] AO_SOFT_RESET, [6] PLL_SOFT_RESET, [1] CLKHV_PHASE_INV, [0] GPIO1_INPUT
 */
#define ES9080Q_SOFT_RESET		0xC0
#define ES9080Q_GPIO1_IN_CLKHV_INV	0x03

/* Private driver state */

struct es9080q_priv {
	struct i2c_client	*i2c;
	struct i2c_client	*i2c_wo;
	struct regmap		*regmap;
	struct regmap		*regmap_wo;
	struct gpio_desc	*reset_gpio;
	struct clk		*mclk;
	bool			hw_inited;
};

/* regmap configurations */

static bool es9080q_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= ES9080Q_RW_REG_MAX || reg >= ES9080Q_RO_REG_MIN;
}

static bool es9080q_writeable_reg(struct device *dev, unsigned int reg)
{
	/* The write-only range is handled by the secondary regmap */
	return reg <= ES9080Q_RW_REG_MAX;
}

static bool es9080q_volatile_reg(struct device *dev, unsigned int reg)
{
	/* Read-only status registers must never be served from the cache */
	return reg >= ES9080Q_RO_REG_MIN;
}

static const struct regmap_config es9080q_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES9080Q_MAX_REG,
	.readable_reg	= es9080q_readable_reg,
	.writeable_reg	= es9080q_writeable_reg,
	.volatile_reg	= es9080q_volatile_reg,
	/*
	 * Use a flat cache so control reads (e.g. amixer enumerating the
	 * volume controls) are served from the cache instead of hitting the
	 * chip. The ES9080Q's I2C port is dead until it is clocked and
	 * initialised, and a NAK storm on this shared bus also breaks the
	 * other codecs (TLV320AIC3106). The driver keeps the map cache-only
	 * until es9080q_hw_init() runs on the first stream.
	 */
	.cache_type	= REGCACHE_FLAT,
};

static bool es9080q_wo_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg >= ES9080Q_WO_REG_MIN && reg <= ES9080Q_WO_REG_MAX;
}

static bool es9080q_wo_readable_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config es9080q_wo_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES9080Q_WO_REG_MAX,
	.readable_reg	= es9080q_wo_readable_reg,
	.writeable_reg	= es9080q_wo_writeable_reg,
	.cache_type	= REGCACHE_NONE,
};

/* Register write helper — selects primary or write-only regmap */

static int es9080q_write(struct es9080q_priv *es, unsigned int reg,
			 unsigned int val)
{
	if (reg >= ES9080Q_WO_REG_MIN && reg <= ES9080Q_WO_REG_MAX)
		return regmap_write(es->regmap_wo, reg, val);
	return regmap_write(es->regmap, reg, val);
}

/* Hardware initialisation sequence */

static int es9080q_hw_init(struct es9080q_priv *es)
{
	int ret;
	int i;

	if (es->reset_gpio) {
		gpiod_set_value_cansleep(es->reset_gpio, 0);
		usleep_range(10000, 15000);
		gpiod_set_value_cansleep(es->reset_gpio, 1);
		usleep_range(10000, 15000);
	}

	/*
	 * PLL/GPIO registers via write-only address. Set the GPIO1 (MCLK) pad
	 * to input mode and invert the CLKHV phase for better DNR.
	 */
	ret = es9080q_write(es, ES9080Q_REG_RESET_PLL1,
			    ES9080Q_GPIO1_IN_CLKHV_INV);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_PLL_CONFIG, 0xC3);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_PLL_PARAMS, 0x40);
	if (ret)
		return ret;

	/* Core registers via primary address */
	ret = es9080q_write(es, ES9080Q_REG_INTERP_MOD_CLK, ES9080Q_ALL_CH_EN);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_TDM_DECODER, ES9080Q_TDM_DECODER_EN);
	if (ret)
		return ret;

	/*
	 * SELECT_IDAC_NUM = MCLK/fs/128 - 1. AUX_MCLK is the AIC3106 PLL output on
	 * GPIO1. Bela's Es9080_Codec.cpp uses SELECT_IDAC_NUM = 3, i.e. MCLK =
	 * 512*fs (GPIO1 = PLL/4, ~24.576 MHz @ 48 kHz). Since we program the
	 * identical AIC3106 GPIO1 value (reg 98 = 0x50), match that with 0x03.
	 * A wrong ratio here makes the DAC interpolator run at the wrong rate.
	 */
	ret = es9080q_write(es, ES9080Q_REG_DAC_CONFIG, 0x03);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_MASTER_CLK, 0x01);
	if (ret)
		return ret;

	/*
	 * CP_CLK_DIV: charge-pump clock = SYS_CLK / ((x+1) * 2), valid 500k-1MHz.
	 * For SYS_CLK = 512*fs (~22.6-24.6 MHz) use 0x0C -> ~870-945 kHz.
	 */
	ret = es9080q_write(es, ES9080Q_REG_CP_CLK_DIV, 0x0C);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_ANALOG_EN, ES9080Q_ALL_CH_EN);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_ANALOG_DELAY, 0xBB);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_PLL_LOCK, ES9080Q_FORCE_PLL_LOCK);
	if (ret)
		return ret;

	/* TDM configuration — slave mode */
	ret = es9080q_write(es, ES9080Q_REG_INPUT_CONFIG, ES9080Q_INPUT_SEL_TDM);
	if (ret)
		return ret;

	/*
	 * MASTER_WS_SCALE only affects master-mode clock generation but is
	 * programmed for a 512*fs MCLK regardless, matching the reference
	 * (MCLK/WS = 512 = menc_div(2) * 128 * 2^WS_SCALE -> WS_SCALE = 1).
	 */
	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG1,
			    ES9080Q_TDM_WS_SCALE_DIV2 | ES9080Q_TDM_CH_NUM(8));
	if (ret)
		return ret;

	/*
	 * Left-justified decode latched on the NEGATIVE WS valid edge. The
	 * frame master emits a 1-BCLK-wide WS pulse, so the falling edge sits
	 * one BCLK after frame start — the same position as the bus's DSP_A
	 * (1-bit-delay) data. Omitting TDM_LJ_MODE misaligns the slot decode
	 * and turns playback into broadband noise.
	 */
	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG2,
			    ES9080Q_TDM_LJ_MODE | ES9080Q_TDM_VALID_EDGE_NEG |
			    ES9080Q_TDM_VALID_PULSE_LEN_8);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_TDM_CONFIG3, ES9080Q_TDM_32BIT);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_BCK_WS_MON, 0x00);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_TDM_VALID_PULSE, 0x00);
	if (ret)
		return ret;

	/* CH1-CH8 to slots 0-7, all on DATA line 1 */
	for (i = 0; i < 8; i++) {
		ret = es9080q_write(es, ES9080Q_REG_TDM_CH1_CFG + i, i);
		if (ret)
			return ret;
	}

	/* Filter and dither */
	ret = es9080q_write(es, ES9080Q_REG_FILTER_CFG, 0x46);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_DITHER_CFG, 0xE4);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_THD_C2_ODD,  0x68);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2H_ODD, 0x01);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C3_ODD,  0x8D);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2_EVEN,  0x68);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C2H_EVEN, 0x01);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_THD_C3_EVEN,  0x8D);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_AUTOMUTE, 0x00);
	if (ret)
		return ret;

	ret = es9080q_write(es, ES9080Q_REG_NSMOD_PHASE,   0xCC);
	if (ret)
		return ret;
	ret = es9080q_write(es, ES9080Q_REG_NSMOD_TYPE,    0x54);
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

	ret = es9080q_write(es, ES9080Q_REG_DRE_CTRL, 0x00);
	if (ret)
		return ret;

	/*
	 * DAC clock resync — 3 sequential writes required to align
	 * all clocks in the DAC core for best analog performance.
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

	/* Power up amplifier */
	ret = es9080q_write(es, ES9080Q_REG_AMP_CTRL, ES9080Q_AMP_ON);
	if (ret)
		return ret;

	/* Force immediate volume updates */
	ret = es9080q_write(es, ES9080Q_REG_VOL_CTRL, ES9080Q_FORCE_VOLUME);
	if (ret)
		return ret;

	/*
	 * Restore the per-channel volumes from the register cache: the chip is
	 * soft-reset on every stream stop, and user control writes issued
	 * while it was in reset only reached the cache. An untouched cache
	 * entry is 0 = 0 dB.
	 */
	for (i = 0; i < 8; i++) {
		unsigned int vol = 0;

		regmap_read(es->regmap, ES9080Q_REG_VOL_CH1 + i, &vol);
		ret = es9080q_write(es, ES9080Q_REG_VOL_CH1 + i, vol);
		if (ret)
			return ret;
	}

	dev_info(&es->i2c->dev, "ES9080Q initialised (8ch TDM slave)\n");
	return 0;
}

/* Component probe */

static int es9080q_probe(struct snd_soc_component *component)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(component);

	/*
	 * The register port is inaccessible until the master clock is running.
	 * No codec is clocked at component-probe (no DAI link is active), so an
	 * I2C access here would NAK (-EREMOTEIO) and tear down the card. Defer
	 * the hardware init to the first hw_params(), once the TDM frame and
	 * master clock are up.
	 */
	es->hw_inited = false;
	dev_dbg(component->dev,
		"registered at I2C 0x%02x (R/W) + 0x%02x (W/O); init deferred to stream start\n",
		es->i2c->addr, es->i2c_wo ? es->i2c_wo->addr : 0);

	return 0;
}

/* DAI operations */

static int es9080q_ensure_init(struct snd_soc_dai *dai)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(dai->component);
	int ret;

	/*
	 * The AIC3106 GPIO1 clock output only starts when its DAPM stream is
	 * powered. ASoC calls digital_mute(false) after that stream event, making
	 * this the first point at which the ES9080Q register port is accessible.
	 * es9080q_hw_init() issues the write-only PLL/reset writes first, which
	 * also bring up the R/W port.
	 */
	if (!es->hw_inited) {
		int attempt;

		regcache_cache_only(es->regmap, false);

		/*
		 * Allow the AIC3106 PLL and its divided GPIO1 output a few
		 * milliseconds to settle before giving up.
		 */
		for (attempt = 0; attempt < 10; attempt++) {
			ret = es9080q_hw_init(es);
			if (!ret)
				break;
			usleep_range(2000, 3000);
		}
		if (ret) {
			/*
			 * Return to cache-only so later control accesses do not
			 * repeatedly NAK on the shared bus while the master clock
			 * is absent.
			 */
			regcache_cache_only(es->regmap, true);
			dev_err(dai->dev,
				"hardware init failed at I2C 0x%02x: %d (master clock running?)\n",
				es->i2c->addr, ret);
			return ret;
		}
		es->hw_inited = true;
	}

	/* The DAC locks to BCLK/WCLK automatically as a TDM consumer. */
	return 0;
}

static int es9080q_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*
	 * The chip is programmed for one fixed framing (es9080q_hw_init()):
	 * left-justified decode on the negative WS valid edge. Against a frame
	 * master emitting a 1-BCLK WS pulse this equals DSP_A (1-bit delay)
	 * with a rising-edge pulse and non-inverted BCLK — DSP_A + IB_NF in
	 * davinci-mcasp terms, whose IB_NF clears the ACLK polarity inversion.
	 * Reject anything else rather than play misaligned audio.
	 */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		break;
	default:
		dev_err(dai->dev, "unsupported DAI format 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		break;
	default:
		dev_err(dai->dev, "unsupported clock inversion 0x%x\n", fmt);
		return -EINVAL;
	}

	/* No master-mode support: BCLK and WCLK must be supplied to the chip */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		dev_err(dai->dev, "unsupported clock provider 0x%x\n", fmt);
		return -EINVAL;
	}

	return 0;
}

static int es9080q_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			      unsigned int freq, int dir)
{
	return 0;
}

static int es9080q_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				unsigned int rx_mask, int slots, int slot_width)
{
	/*
	 * The chip decodes a fixed frame: 8 slots x 32 bits on DATA line 1,
	 * channel N from slot N (programmed in es9080q_hw_init()). Accept only
	 * that layout; the tx_mask may be a subset when fewer outputs are fed.
	 */
	if (slots != 8 || slot_width != 32) {
		dev_err(dai->dev, "unsupported TDM geometry %dx%d (need 8x32)\n",
			slots, slot_width);
		return -EINVAL;
	}

	if (!tx_mask || tx_mask & ~0xffU || rx_mask) {
		dev_err(dai->dev, "unsupported TDM masks tx 0x%x rx 0x%x\n",
			tx_mask, rx_mask);
		return -EINVAL;
	}

	return 0;
}

static int es9080q_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct es9080q_priv *es = snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (mute) {
		if (!es->hw_inited)
			return 0;

		/*
		 * Stream stop. The R/W register port may already be dead here:
		 * it needs a running MCLK, and AUX_MCLK stops with the AIC3106
		 * stream. Soft-reset the chip through the WRITE-ONLY port
		 * instead, which works without MCLK (this is also what Bela's
		 * reference driver does on stop). The next stream start runs
		 * the full init again; meanwhile serve controls from the
		 * cache so user writes cannot NAK on the shared bus.
		 */
		ret = es9080q_write(es, ES9080Q_REG_RESET_PLL1,
				    ES9080Q_SOFT_RESET);
		if (!ret)
			ret = es9080q_write(es, ES9080Q_REG_RESET_PLL1, 0x00);

		regcache_cache_only(es->regmap, true);
		es->hw_inited = false;
		return ret;
	}

	ret = es9080q_ensure_init(dai);
	if (ret)
		return ret;

	/*
	 * The full 8-slot frame is always driven (the DAI's TDM mask fixes the
	 * codec up to 8 channels whatever the stream width), so all channels
	 * stay enabled; only the amp state machine is gated per stream.
	 */
	return regmap_write(es->regmap, ES9080Q_REG_AMP_CTRL, ES9080Q_AMP_ON);
}

static const struct snd_soc_dai_ops es9080q_dai_ops = {
	.set_fmt	 = es9080q_set_fmt,
	.set_sysclk	 = es9080q_set_sysclk,
	.set_tdm_slot	 = es9080q_set_tdm_slot,
	.mute_stream	 = es9080q_mute_stream,
	.no_capture_mute = 1,
};

/* DAI driver */

static struct snd_soc_dai_driver es9080q_dai = {
	.name = "es9080q-hifi",
	.playback = {
		.stream_name  = "ES9080Q Playback",
		.channels_min = 1,
		.channels_max = 8,
		/*
		 * The init sequence hardcodes the 512*fs MCLK ratio
		 * (SELECT_IDAC_NUM) that the Bela Gem clock tree provides at
		 * these rates. Higher rates need a rate-dependent IDAC divider
		 * and a faster MCLK; do not advertise them until implemented.
		 */
		.rates        = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats      = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es9080q_dai_ops,
};

/* DAPM and mixer controls */

/*
 * Volume TLV: 0 dB to -127.5 dB in 0.5 dB steps.
 * Register value = attenuation in half-dB units (0 = 0 dB, 255 = -127.5 dB).
 */
static const DECLARE_TLV_DB_SCALE(es9080q_vol_tlv, -12750, 50, 0);

static const struct snd_kcontrol_new es9080q_controls[] = {
	SOC_SINGLE_TLV("Ch1 Playback Volume", ES9080Q_REG_VOL_CH1, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch2 Playback Volume", ES9080Q_REG_VOL_CH2, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch3 Playback Volume", ES9080Q_REG_VOL_CH3, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch4 Playback Volume", ES9080Q_REG_VOL_CH4, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch5 Playback Volume", ES9080Q_REG_VOL_CH5, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch6 Playback Volume", ES9080Q_REG_VOL_CH6, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch7 Playback Volume", ES9080Q_REG_VOL_CH7, 0, 255, 1, es9080q_vol_tlv),
	SOC_SINGLE_TLV("Ch8 Playback Volume", ES9080Q_REG_VOL_CH8, 0, 255, 1, es9080q_vol_tlv),

	/*
	 * Master switch = the AMP power-up state machine (AMP_CTRL bit 1), the
	 * same bit mute_stream() drives. Starting or stopping a stream therefore
	 * overrides whatever the user last set here.
	 */
	SOC_SINGLE("Master Playback Switch", ES9080Q_REG_AMP_CTRL, 1, 1, 0),

	/*
	 * Per-channel analog-section enables (ANALOG_EN bits 0-7). mute_stream()
	 * rewrites this register on unmute to enable exactly the channels the TDM
	 * frame carries, so per-channel switches set here do not survive a stream
	 * start.
	 */
	SOC_SINGLE("Ch1 Playback Switch", ES9080Q_REG_ANALOG_EN, 0, 1, 0),
	SOC_SINGLE("Ch2 Playback Switch", ES9080Q_REG_ANALOG_EN, 1, 1, 0),
	SOC_SINGLE("Ch3 Playback Switch", ES9080Q_REG_ANALOG_EN, 2, 1, 0),
	SOC_SINGLE("Ch4 Playback Switch", ES9080Q_REG_ANALOG_EN, 3, 1, 0),
	SOC_SINGLE("Ch5 Playback Switch", ES9080Q_REG_ANALOG_EN, 4, 1, 0),
	SOC_SINGLE("Ch6 Playback Switch", ES9080Q_REG_ANALOG_EN, 5, 1, 0),
	SOC_SINGLE("Ch7 Playback Switch", ES9080Q_REG_ANALOG_EN, 6, 1, 0),
	SOC_SINGLE("Ch8 Playback Switch", ES9080Q_REG_ANALOG_EN, 7, 1, 0),
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

static const struct snd_soc_component_driver es9080q_component_driver = {
	.probe			= es9080q_probe,
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

/* I2C driver */

static int es9080q_i2c_probe(struct i2c_client *i2c)
{
	struct es9080q_priv *es;
	u32 wo_addr;

	es = devm_kzalloc(&i2c->dev, sizeof(*es), GFP_KERNEL);
	if (!es)
		return -ENOMEM;

	es->i2c = i2c;
	i2c_set_clientdata(i2c, es);

	es->regmap = devm_regmap_init_i2c(i2c, &es9080q_regmap_config);
	if (IS_ERR(es->regmap))
		return dev_err_probe(&i2c->dev, PTR_ERR(es->regmap),
				     "failed to init primary regmap\n");

	/*
	 * Serve register access from the cache until the chip is clocked and
	 * initialised (cleared in es9080q_hw_init() on the first stream). This
	 * keeps control-read NAKs off the shared I2C bus, where they would also
	 * disturb the other codecs.
	 */
	regcache_cache_only(es->regmap, true);

	/*
	 * The register interface is inaccessible until the master clock is
	 * running. Enable the optional "mclk" so it is up before the init
	 * sequence; a board may instead gate this clock from the machine driver.
	 */
	es->mclk = devm_clk_get_optional_enabled(&i2c->dev, "mclk");
	if (IS_ERR(es->mclk))
		return dev_err_probe(&i2c->dev, PTR_ERR(es->mclk),
				     "failed to get/enable mclk\n");

	/*
	 * Set up the write-only I2C client for PLL/reset registers.
	 * "ess,write-only-addr" specifies the secondary address (primary + 4).
	 */
	if (!of_property_read_u32(i2c->dev.of_node, "ess,write-only-addr",
				  &wo_addr)) {
		es->i2c_wo = devm_i2c_new_dummy_device(&i2c->dev,
						       i2c->adapter,
						       wo_addr);
		if (IS_ERR(es->i2c_wo))
			return dev_err_probe(&i2c->dev, PTR_ERR(es->i2c_wo),
					     "failed to create write-only I2C client at 0x%02x\n",
					     wo_addr);

		es->regmap_wo = devm_regmap_init_i2c(es->i2c_wo,
						     &es9080q_wo_regmap_config);
		if (IS_ERR(es->regmap_wo))
			return dev_err_probe(&i2c->dev, PTR_ERR(es->regmap_wo),
					     "failed to init write-only regmap\n");

		dev_info(&i2c->dev,
			 "ES9080Q: primary 0x%02x, write-only 0x%02x\n",
			 i2c->addr, wo_addr);
	} else {
		dev_warn(&i2c->dev,
			 "ES9080Q: no write-only addr, PLL config unavailable\n");
		/* Point write-only regmap at primary to avoid NULL deref */
		es->regmap_wo = es->regmap;
	}

	/*
	 * Optional CHIP_EN / reset GPIO. If not wired, driver proceeds
	 * without hardware reset.
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
