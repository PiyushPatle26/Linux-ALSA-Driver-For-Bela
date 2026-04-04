# Linux ALSA Driver for Bela (PB2 / AM62x)

## Project

GSoC project: upstream Linux ALSA/ASoC support for Bela Gem audio capes
on PocketBeagle2 (AM62x).

Two boards:
- **Bela Gem Stereo** — TLV320AIC3106 only (2ch playback + 2ch capture)
- **Bela Gem Multi** — TLV320AIC3106 + ES9080Q + 2x TLV320ADC3140 (10ch playback + 4ch capture)

## Repository Structure

```
driver/sound/soc/codecs/es9080q.c          ES9080Q 8-ch DAC codec driver (new)
driver/sound/soc/codecs/Kconfig            Kconfig entry for ES9080Q
driver/sound/soc/codecs/Makefile           Makefile entry for ES9080Q
driver/sound/soc/ti/bela.c                 Bela machine driver (Multi, AM62x port pending)
driver/sound/soc/ti/Kconfig                Kconfig entry for Bela machine driver
driver/sound/soc/ti/Makefile               Makefile entry for Bela machine driver

dts/k3-am62-pocketbeagle2-bela-gem-stereo.dtso   Stereo overlay (simple-audio-card)
dts/k3-am62-pocketbeagle2-bela-gem-multi.dtso     Multi overlay (multi-DAI-link)
dts/k3-am62-pocketbeagle2-bela-gem.dtsi           Reference dtsi (compiled into base DT)
dts/k3-am62-pocketbeagle2.dts                     Base PB2 DT (includes dtsi)
dts/BB-BONE-BELA-REVC-00A0.dts                    Legacy BBB Bela cape (reference only)

Documentation/devicetree/bindings/sound/ess,es9080q.yaml       ES9080Q DT binding
Documentation/devicetree/bindings/sound/bela,audio-cape.yaml    Bela Multi machine DT binding

BRINGUP.md              Step-by-step hardware bring-up guide
hardware_validation.md  Oscilloscope/logic-analyzer validation phases
UPSTREAM-STRUCTURE.txt  Upstream patch series file layout
```

## Pin Mapping (Verified)

From `k3-am62-pocketbeagle2-bela-gem.dtsi` and GemMulti rev A4 schematic:

| Signal | Pin | McASP2 Function | Direction |
|--------|------|-----------------|-----------|
| AUD_DIN  | P2.05 | AXR0 | TX (SoC → codec DIN, playback) |
| AUD_DOUT | P2.07 | AXR1 | RX (codec DOUT → SoC, capture) |
| AUD_WCLK | P2.10 | AFSX | Input (codec-driven) |
| AUD_MCLK | P2.11 | AUDIO_EXT_REFCLK1 | Output (12.288 MHz) |
| AUD_BCLK | P2.19 | ACLKX | Input (codec-driven) |

I2C bus: `main_i2c1` (`/dev/i2c-2`)

| Device | I2C Address | Notes |
|--------|-------------|-------|
| TLV320AIC3106 | 0x18 | BCLK/WCLK master |
| ES9080Q (R/W) | 0x48 | Primary registers 0–164 |
| ES9080Q (W/O) | 0x4C | Reset/PLL registers 192–203 |

## Current Status

- ES9080Q codec driver: register map verified, init sequence implemented
- Stereo overlay: ready for hardware testing (uses upstream drivers only)
- Multi overlay: ready for testing after Stereo is validated
- Machine driver (`bela.c`): AM62x port pending (uses AM335x clock model)
- Next milestone: Stereo board audio test on hardware

## Bring-Up Order

1. **Stereo board** — follow `BRINGUP.md` Board A Steps 1–5
2. **Multi board** — follow `BRINGUP.md` Board B Steps 1–6
3. Port `bela.c` machine driver to AM62x (future)
