# Linux ALSA/ASoC support for Bela Gem audio capes (PocketBeagle2 / AM62x)

This repository hosts the in-development Linux kernel support for the
[Bela](https://bela.io) Gem audio capes on the PocketBeagle2 (TI AM62x / AM625).
It is being prepared for upstream submission to the ALSA SoC (ASoC) subsystem.

Two cape variants are targeted:

| Cape | Codecs | Audio I/O |
|------|--------|-----------|
| **Bela Gem Stereo** | TLV320AIC3106 | 2-ch playback + 2-ch capture |
| **Bela Gem Multi** | TLV320AIC3106 + ES9080Q + 2× TLV320ADC3140 | 10-ch playback + 10-ch capture |

All audio flows over a single McASP2 instance. The TLV320AIC3106 is the BCLK/WCLK
master for the whole frame; every other codec is a clock slave.

## Status

| Component | State |
|-----------|-------|
| Stereo overlay (upstream `simple-audio-card` + `tlv320aic3x`) | **Validated on hardware** — playback and capture working |
| ES9080Q codec driver (`es9080q.c`) | Register map + dual-I2C verified against Bela userspace driver; builds clean; pending on-hardware bring-up |
| Multi machine driver (`am625-bela-gem.c`) | AM62x driver written, builds clean, probes; pending on-hardware bring-up |
| Multi overlay | Builds clean; codecs ACK on I2C except ES9080Q (see hardware note) |

**Hardware note (Multi):** the ES9080Q (`0x48`) does not appear on I2C until the
board-level solder jumper **JP8** (ES9080Q `CHIP_EN` → +3V3) is bridged. Until then
the Multi card will not register.

### Known gaps before upstream submission

- `am625-bela-gem.c` requires five phandles (`cpu-dai`, `aic3106-codec`,
  `es9080q-codec`, `adc3140-1-codec`, `adc3140-2-codec`), but the binding
  `ti,am625-bela-gem-multi.yaml` currently documents only the first three. The
  binding must be extended to cover the two ADC3140 capture codecs before the
  series is sent.
- The legacy AM335x machine driver `driver/sound/soc/ti/bela.c` is kept as a
  reference only; it is not part of the AM62x target and is not built.
- On-hardware validation of the Multi card is blocked on JP8 (above).

## Repository layout

```
driver/sound/soc/codecs/
  es9080q.c                              ESS ES9080Q 8-ch DAC codec driver (NEW)
  Kconfig, Makefile                      ES9080Q build integration

driver/sound/soc/ti/
  am625-bela-gem.c                       Bela Gem Multi ASoC machine driver (NEW)
  bela.c                                 Legacy AM335x machine driver (reference only)
  Kconfig, Makefile                      Machine-driver build integration

dts/
  k3-am62-pocketbeagle2-bela-gem-stereo.dtso   Stereo cape overlay (simple-audio-card)
  k3-am62-pocketbeagle2-bela-gem-multi.dtso    Multi cape overlay (am625-bela-gem)
  k3-am62-pocketbeagle2-bela-gem.dtsi          Shared cape reference dtsi
  k3-am62-pocketbeagle2-bela-gem-stereo.dtsi   Stereo reference dtsi
  BB-BONE-BELA-REVC-00A0.dts                   Legacy BeagleBone Black cape (reference)

Documentation/devicetree/bindings/sound/
  ess,es9080q.yaml                       ES9080Q codec DT binding (NEW)
  ti,am625-bela-gem-multi.yaml           Bela Gem Multi machine DT binding (NEW)
  bela,audio-cape.yaml                   Earlier cape binding draft
```

## Hardware configuration

### McASP2 pin map (PocketBeagle2 headers → AM62x pads)

Every device tree in this repository uses the same serializer direction:
`serial-dir = <1 2 ...>` — **AXR0 = TX (playback), AXR1 = RX (capture)**. This is
the direction validated on hardware with the Stereo cape.

| Signal | Header | McASP2 function | Direction |
|--------|--------|-----------------|-----------|
| AUD_DIN  | P2.05 | AXR0 | TX — SoC → codec DIN (playback) |
| AUD_DOUT | P2.07 | AXR1 | RX — codec DOUT → SoC (capture) |
| ES9080Q data | P1.04 | AXR6 | TX — SoC → ES9080Q (8-ch, single data line) |
| ADC3140 #1 data | P1.02 | AXR4 | RX — ADC3140 #1 → SoC (Multi only) |
| ADC3140 #2 data | P2.31 | AXR8 | RX — ADC3140 #2 → SoC (Multi only) |
| AUD_WCLK | P2.10 | AFSX  | Input (codec-driven) |
| AUD_BCLK | P2.19 | ACLKX | Input (codec-driven) |
| AUD_MCLK | P2.11 | AUDIO_EXT_REFCLK1 | Output (~24 MHz) |

> Bela's downstream `PB2-BELA.dtso` uses the reverse mapping (`serial-dir = <2 1>`).
> This repository deliberately keeps `<1 2>` because it is the direction confirmed
> working on hardware. Do not flip it without re-validating on a board.

### I2C devices (`main_i2c1`, exposed as `/dev/i2c-2`)

| Device | Address | Role |
|--------|---------|------|
| TLV320AIC3106 | 0x18 | Stereo codec, BCLK/WCLK master |
| ES9080Q | 0x48 (R/W), 0x4C (write-only) | 8-ch DAC, clock slave (Multi) |
| TLV320ADC3140 #1 | 0x4E | 4-ch ADC, clock slave (Multi) |
| TLV320ADC3140 #2 | 0x4F | 4-ch ADC, clock slave (Multi) |

### Measured clock baseline (Stereo, under Bela userspace runtime)

| Signal | Pin | Measured |
|--------|-----|----------|
| MCLK | P2.11 | ~24 MHz (96 MHz / 4) |
| BCLK | P2.19 | ~1.41 MHz (44100 × 2 × 16) |
| Format | — | I2S, 16-bit, 44.1 kHz (Stereo) |

## Building

The drivers and overlays build out-of-tree against a configured AM62x arm64
kernel tree (mainline / linux-next with the AM62x McASP support present).

```sh
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-

# Copy the sources into your kernel tree (paths mirror the in-kernel layout):
#   driver/sound/soc/codecs/es9080q.c        -> sound/soc/codecs/
#   driver/sound/soc/ti/am625-bela-gem.c     -> sound/soc/ti/
#   dts/*.dtso                               -> arch/arm64/boot/dts/ti/
#   Documentation/.../*.yaml                 -> Documentation/devicetree/bindings/sound/
# and apply the matching Kconfig/Makefile hunks.

# Enable as modules:
#   CONFIG_SND_SOC_ES9080Q=m
#   CONFIG_SND_SOC_AM625_BELA_GEM=m
#   CONFIG_SND_SOC_TLV320AIC3X_I2C=m
#   CONFIG_SND_SOC_TLV320ADCX140=m

make M=sound/soc/codecs modules
make M=sound/soc/ti      modules
make ti/k3-am62-pocketbeagle2-bela-gem-stereo.dtbo
make ti/k3-am62-pocketbeagle2-bela-gem-multi.dtbo
```

### Validate the bindings

```sh
make dt_binding_check \
  DT_SCHEMA_FILES=Documentation/devicetree/bindings/sound/ess,es9080q.yaml
make dt_binding_check \
  DT_SCHEMA_FILES=Documentation/devicetree/bindings/sound/ti,am625-bela-gem-multi.yaml
```

## On-target bring-up

```sh
# 1. I2C sanity before applying the overlay
i2cdetect -y 2            # Stereo: expect 0x18.  Multi: 0x18, 0x48, 0x4e, 0x4f

# 2. Apply the overlay (configfs)
mkdir -p /sys/kernel/config/device-tree/overlays/bela
cat k3-am62-pocketbeagle2-bela-gem-stereo.dtbo \
    > /sys/kernel/config/device-tree/overlays/bela/dtbo
dmesg | tail            # expect "OF: overlay: ... applied"

# 3. Load modules and check the card
modprobe snd-soc-tlv320aic3x-i2c
modprobe snd-soc-davinci-mcasp
modprobe snd-soc-simple-card      # Stereo
# modprobe snd-soc-es9080q snd-soc-am625-bela-gem   # Multi
aplay -l
cat /proc/asound/cards

# 4. Audio test (Stereo baseline: 44.1 kHz)
speaker-test -D hw:BelaGemStereo -c 2 -t sine -f 440 -r 44100
arecord -D hw:BelaGemStereo -f S16_LE -r 44100 -c 2 -d 5 /tmp/cap.wav
aplay   -D hw:BelaGemStereo /tmp/cap.wav
```

## Planned upstream patch series

Submission order:

1. `dt-bindings: sound: Add binding for ESS ES9080Q DAC`
2. `ASoC: codecs: Add ESS ES9080Q 8-channel DAC driver`
3. `dt-bindings: sound: Add Bela Gem Multi sound card binding`
4. `ASoC: ti: Add Bela Gem Multi machine driver`
5. `arm64: dts: ti: Add Bela Gem Stereo overlay for PocketBeagle2`
6. `arm64: dts: ti: Add Bela Gem Multi overlay for PocketBeagle2`
7. `MAINTAINERS: Add entry for Bela Gem audio (PocketBeagle2 / AM62x)`
