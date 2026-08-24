# Linux ALSA/ASoC support for Bela Gem audio capes (PocketBeagle2 / AM62x)

This repository hosts the in-development Linux kernel support for the
[Bela](https://bela.io) Gem audio capes on the PocketBeagle2 (TI AM62x / AM625).
It is being prepared for upstream submission to the ALSA SoC (ASoC) subsystem.

Two cape variants are targeted:

| Cape | Codecs | Audio I/O |
|------|--------|-----------|
| **Bela Gem Stereo** | TLV320AIC3106 | 2-ch playback + 2-ch capture |
| **Bela Gem Multi** | TLV320AIC3106 + ES9080Q + 2× TLV320ADC3140 | 10 outputs + 10 inputs |

All audio flows over a single McASP2 instance. The TLV320AIC3106 is the
BCLK/WCLK (frame) master: its PLL generates BCLK (256 clocks/frame), WCLK, and
AUX_MCLK on its GPIO1 pad — the master clock for the ES9080Q and both
ADC3140s. Deriving BCLK and AUX_MCLK from the one PLL is required because the
ES9080Q needs an integer MCLK:BCLK ratio. McASP2 and all other codecs are
clock consumers. This matches Bela's shipping userspace configuration.

## Status

| Component | State |
|-----------|-------|
| Stereo overlay (upstream `simple-audio-card` + `tlv320aic3x`) | **Validated on hardware** — playback and capture working |
| Multi card registration | **Working** — `BelaGemMulti` registers with 1 playback + 3 capture PCM devices |
| ES9080Q codec driver (`es9080q.c`) | **Initialises on hardware** (`ES9080Q initialised (8ch TDM slave)`) once the AIC3106 PLL provides AUX_MCLK |
| 16-channel playback frame | Implemented (see channel map below); **hardware validation pending** |
| Audio quality on ES9080Q outputs | **Open bug** — broadband noise reported; suspects are bit delay / clock polarity (see Known issues) |
| AIC3106 register access during 8-ch streams | **Open bug** — `-EREMOTEIO` storm after ES9080Q powers up (see Known issues) |
| ADC3140 capture | PCM devices enumerate; **streaming unverified** (serializer routing, see Known issues) |

## Playback channel map (Multi)

davinci-mcasp interleaves a multi-serializer stream round-robin: channel *n*
goes to serializer *(n % active)*, slot *(n / active)*. With both TX
serializers active (a 16-channel stream):

| Stream channels | Serializer | Destination |
|---|---|---|
| 0, 2 | AXR0 slots 0-1 | AIC3106 left, right |
| 1, 3, 5, 7, 9, 11, 13, 15 | AXR6 slots 0-7 | ES9080Q OUT1–OUT8 |
| 4, 6, 8, 10, 12, 14 | AXR0 slots 2-7 | unused (AIC3106 decodes slots 0-1 only) |

A 1- or 2-channel stream activates AXR0 alone and plays through the AIC3106.
The machine driver constrains playback to **{1, 2, 16}** channels: counts of
3–8 would fit a single serializer and land entirely on AXR0, never reaching
the DAC.

```sh
# stereo through the AIC3106
speaker-test -D hw:BelaGemMulti,0 -c 2 -t sine -f 440 -r 48000 -F S32_LE
# all ten outputs (16-ch interleaved stream)
speaker-test -D hw:BelaGemMulti,0 -c 16 -t sine -f 440 -r 48000 -F S32_LE
```

## Repository layout

```
driver/sound/soc/codecs/
  es9080q.c                              ESS ES9080Q 8-ch DAC codec driver (NEW)
  Kconfig, Makefile                      ES9080Q build integration

driver/sound/soc/ti/
  am625-bela-gem.c                       Bela Gem Multi ASoC machine driver (NEW)
  Kconfig, Makefile                      Machine-driver build integration

dts/
  k3-am62-pocketbeagle2-bela-gem-stereo.dtso   Stereo cape overlay (simple-audio-card)
  k3-am62-pocketbeagle2-bela-gem-multi.dtso    Multi cape overlay (am625-bela-gem)

Documentation/devicetree/bindings/sound/
  ess,es9080q.yaml                       ES9080Q codec DT binding (NEW)
  ti,am625-bela-gem-multi.yaml           Bela Gem Multi machine DT binding (NEW)

MAINTAINERS                              Entry for the upstream series (patch 8/8)
linux/                                   Kernel tree the above is developed in (untracked)

build.sh                                 Build modules + overlays with config/vermagic guards
shrink-config.sh                         Trim .config from arm64 defconfig to AM62x + Bela
deploy.sh                                Deploy Image/modules/DTB/overlays to the SD card
```

## Hardware configuration

### McASP2 pin map (PocketBeagle2 headers → AM62x pads)

Multi overlay serializer directions (`serial-dir`): **AXR0=TX, AXR1=RX,
AXR4=RX, AXR6=TX, AXR8=RX**.

| Signal | Header | McASP2 function | Direction |
|--------|--------|-----------------|-----------|
| AUD_DIN  | P2.05 | AXR0 | TX — SoC → AIC3106 DIN |
| AUD_DOUT | P2.07 | AXR1 | RX — AIC3106 DOUT → SoC |
| ES9080Q data | P1.04 | AXR6 | TX — SoC → ES9080Q (8-ch, single data line) |
| ADC3140 #1 data | P1.02 | AXR4 | RX — ADC3140 #1 → SoC |
| ADC3140 #2 data | P2.31 | AXR8 | RX — ADC3140 #2 → SoC |
| AUD_WCLK | P2.10 | AFSX  | **Input** (AIC3106 is frame master) |
| AUD_BCLK | P2.19 | ACLKX | **Input** (AIC3106 is bit-clock master) |
| AUD_MCLK | P2.11 | AUDIO_EXT_REFCLK1 | Output (24 MHz → AIC3106 MCLK) |

### I2C devices (`main_i2c1` — Linux bus number varies by kernel config, run `i2cdetect -l` first)

| Device | Address | Role |
|--------|---------|------|
| TLV320AIC3106 | 0x18 | Stereo codec, BCLK/WCLK master, AUX_MCLK source |
| ES9080Q | 0x48 (R/W), 0x4C (write-only) | 8-ch DAC, clock consumer (Multi) |
| TLV320ADC3140 #1 | 0x4E | 4-ch ADC, clock consumer (Multi) |
| TLV320ADC3140 #2 | 0x4F | 4-ch ADC, clock consumer (Multi) |

The ES9080Q's R/W register port only responds while its MCLK is running, and
`i2cdetect` shows `UU` for bound drivers — a bound ES9080Q is *not* evidence
that MCLK is present. The driver defers all register access to the first
stream for this reason.

## Building

```sh
# one-time: .config from arm64 defconfig, trimmed to AM62x + Bela
./shrink-config.sh

# modules + overlays (guards kernelrelease and the Bela config)
./build.sh

# or by hand:
cd linux
make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LOCALVERSION= Image modules dtbs
```

Validate the bindings before sending:

```sh
make ARCH=arm64 dt_binding_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/sound/ess,es9080q.yaml
make ARCH=arm64 dt_binding_check DT_SCHEMA_FILES=Documentation/devicetree/bindings/sound/ti,am625-bela-gem-multi.yaml
scripts/checkpatch.pl --strict -g HEAD
```

## Deploying and booting

```sh
sudo ./deploy.sh          # Image + full-or-incremental modules + base DTB + overlays
```

The extlinux labels `bela` (Stereo) and `bela-multi` (Multi) boot
`/bela/Image` with the base DTB and the matching overlay applied **by U-Boot**
(`fdtoverlays`). Runtime overlay loading via configfs is *not* available: the
`OF_CONFIGFS` patch is BeagleBoard-downstream and not in mainline. If the boot
drops into NuttX/NSH instead of the menu, remove or rename `uEnv.txt` on the
boot partition — its `uenvcmd` jumps to `nuttx.bin` before extlinux is read.

## Known issues

1. **ES9080Q audio quality (root cause identified, fix pending hardware
   test)** — playback was observed as broadband noise. Auditing
   `es9080q_hw_init()` against Bela's `Es9080_Codec.cpp` found TDM CONFIG2
   (reg 0x50) written as `0x08` where the reference writes `0x88`: the
   **TDM_LJ_MODE bit was missing**, so the slot decode was misaligned — Bela's
   maintainer had flagged misalignment/polarity as the noise signature. The
   driver now writes the full reference value (LJ mode, negative WS valid
   edge, pulse len 8), which pairs with the bus's DSP_A 1-bit delay: the
   falling edge of the 1-BCLK WS pulse lands exactly one BCLK after frame
   start. McASP polarity was confirmed correct (Bela's register dump shows
   `aclkxctl=0`, which davinci-mcasp's `IB_NF` reproduces).
2. **AIC3106 `-EREMOTEIO` storm** — after the ES9080Q's analog section and
   charge pump power up mid-stream, the whole I2C bus stops ACKing (AIC3106
   first, at stream stop). Suspected supply droop (all-8-channel DAC power-on)
   — not yet isolated; test with the Stereo overlay (no ES9080Q) and a proper
   5 V supply. Two software contributions to the storm are fixed: the ES9080Q
   now soft-resets through its write-only port on stream stop (which works
   without MCLK) instead of NAKing on the R/W port, and the ADC3140s are no
   longer told to master BCLK/WCLK against the AIC3106 (per-component
   `ext_fmt` clock roles).
3. **ADC3140 capture routing** — all three capture PCMs enumerate, but
   davinci-mcasp activates RX serializers in index order, so a 4-ch capture on
   the ADC links may read AXR1 (the AIC3106's pin) rather than AXR4/AXR8.
   Streaming from devices 1 and 2 is unverified. The two ADCs are on separate
   data lines by design (R56 DNP — lower bit clock, less EMI).

## Planned upstream patch series

1. `dt-bindings: vendor-prefixes: Add ess (ESS Technology, Inc.)`
2. `dt-bindings: sound: Add binding for ESS ES9080Q DAC`
3. `ASoC: codecs: Add ESS ES9080Q 8-channel DAC driver`
4. `dt-bindings: sound: Add Bela Gem Multi sound card binding`
5. `ASoC: ti: Add Bela Gem Multi machine driver`
6. `arm64: dts: ti: Add Bela Gem Stereo overlay for PocketBeagle2`
7. `arm64: dts: ti: Add Bela Gem Multi overlay for PocketBeagle2`
8. `MAINTAINERS: Add entry for Bela Gem audio (PocketBeagle2 / AM62x)`

Vendor prefix: **confirmed ESS** against the ES9080 datasheet (`ES9080.PDF`,
repo root) — "ESS' patented HyperStream® II architecture", "Sabre®". The
proposal bibliography's "Everest Semi" credit was an error; `ess,` is correct
and patch 1 registers it in `vendor-prefixes.yaml`.
