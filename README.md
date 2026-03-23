# Linux ALSA Driver for Bela (PB2 / AM62x)

## Current Progress

This project is an ongoing bring-up of the **Bela Gem Multi audio system** on **PocketBeagle 2 (AM62x)**.

The codebase currently includes:

* ES9080Q codec driver (functional, SoC-independent)
* Bela machine driver (AM335x-oriented, not yet ported)
* AM335x DTS (used as reference only)
* Initial devicetree bindings for both codecs

Reference: 

---

## Implemented Components

### 1. ES9080Q Codec Driver

* Fully implemented ALSA SoC codec driver
* Features:

  * I2C communication including write-only address handling
  * 8-channel DAC support via TDM
  * Per-channel volume control (`SOC_SINGLE_EXT`)
* Designed for multi-codec TDM systems

Status:
Functionally complete. Needs validation on real hardware.

---

### 2. Machine Driver (`bela_cape.c`)

Defines the intended audio topology:

```
AXR3 → TLV320AIC3106 → slots 0–1
AXR2 → ES9080Q       → slots 2–9
```

Includes:

* Multi-codec DAI links
* TDM slot separation logic
* MCLK switching based on sample rate
* Basic DAPM routing

Limitation:

* Written for AM335x McASP
* Not compatible with AM62x without modification

---

### 3. Device Tree (Reference Only)

`BB-BONE-BELA-REVC-00A0.dts`:

* Describes working system on AM335x
* Used to understand:

  * McASP configuration
  * Codec routing
  * Clocking assumptions

Not usable on PocketBeagle 2.

---

## Architecture (Intended)

Single McASP instance with TDM split across two codecs:

```
McASP0 (TDM bus)

  slots 0–1  → AIC3106 (2-channel ADC/DAC)
  slots 2–9  → ES9080Q (8-channel DAC)
```

Key point:
This is not mixing. It is strict slot partitioning on the same TDM stream.

---

## Current Gaps

### 1. PB2 Device Tree Overlay — INCOMPLETE

No working AM62x overlay exists yet.

Missing:

* `main_pmx0` pinmux configuration
* Correct McASP0 serializer mapping
* AHCLKX (MCLK) configuration
* Proper `k3_clks` usage
* Verified I2C bus binding

---

### 2. Hardware Validation Not Done

The following are still assumptions:

* Which I2C bus the codecs are on
* Whether codec is clock master (FSYNC/BCLK direction)
* AHCLKX routing (critical for PLL lock)
* ES9080Q reset GPIO

These directly affect whether the system will work at all.

---

### 3. Machine Driver Not Ported

Current issues:

* Uses AM335x clock model
* Uses legacy pinmux assumptions
* No integration with AM62x clock framework (`k3_clks`)

---

## What is Confirmed

From PB2 expansion documentation:

| Signal | Pin   | Function     |
| ------ | ----- | ------------ |
| AXR0   | P1.02 | RX           |
| AFSX   | P1.04 | FSYNC        |
| AFSR   | P1.06 | RX FSYNC     |
| ACLKR  | P1.08 | BCLK         |
| AXR3   | P1.10 | TX (AIC3106) |
| AXR2   | P1.12 | TX (ES9080Q) |

Codec:

* TLV320AIC3106 (not AIC3104)
* ES9080Q

---

## Critical Assumptions (Unverified)

1. Codec is clock master

   * McASP receives BCLK + FSYNC

2. TDM slot mapping is valid

   * AXR3 → slots 0–1
   * AXR2 → slots 2–9

3. MCLK is available

   * AHCLKX source not identified

If any of these are wrong, audio will fail even if the driver is correct.

---

## Immediate Next Steps

### Step 1 — Verify Hardware (mandatory)

```
i2cdetect -l
i2cdetect -y <bus>
```

Expected:

* 0x18 → AIC3106
* 0x48 → ES9080Q

If not visible, stop. Fix hardware understanding first.

---

### Step 2 — Minimal AM62x Overlay

Goal: probe codecs only

```
&main_i2cX {
    tlv320aic3106@18 { ... };
    es9080q@48 { ... };
};
```

No McASP yet.

Success = probe messages in dmesg

---

### Step 3 — Validate ES9080Q Driver

* Load module
* Confirm:

  * Probe success
  * Register writes succeed

---

### Step 4 — Bring Up McASP (AM62x)

Port:

* Pinmux → `main_pmx0`
* Clocks → `k3_clks`
* Serializers → AXR2 / AXR3

Goal:

* McASP active
* No clock errors

---

### Step 5 — Port Machine Driver

* Replace AM335x clock logic
* Adapt DAI links for AM62x
* Verify DAPM routing

---

### Step 6 — Audio Testing

Order matters:

1. AIC3106 (2-channel)
2. ES9080Q (8-channel)
3. Full 10-channel TDM

## Summary

* ES9080Q driver: ready
* Machine driver: not portable yet
* DTS: incomplete for PB2
* Hardware mapping: partially known
* Next milestone: successful codec probe on AM62x

---

