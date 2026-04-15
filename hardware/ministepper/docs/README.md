# Tsumikoro TMC2130 Dual Stepper Driver — Hardware

4-layer PCB implementing a dual-axis stepper controller node. One
STM32G071G8U6 MCU drives two TMC2130 stepper drivers over shared SPI,
talks to the RS-485 bus at 1 Mbaud, and accepts motor power from either
the bus V+ rail or an external JST-XH power input (diode-OR'd).

Target MCU matches `firmware/tsumikoro-ministepper/`. Target
manufacturing: **JLCPCB 4-layer (JLC04161H)**, Standard assembly.

![3D top view](images/pcb-3d-top.png)

## Block overview

| Block | Part | Notes |
|-------|------|-------|
| MCU | STM32G071G8U6 (UFQFPN-28) | ARM Cortex-M0+ @ 64 MHz, 64 KB flash, 36 KB RAM — same as firmware target |
| Stepper drivers | 2 × TMC2130-LA-T (QFN-36 5×6) | 2 A per phase, 46 V max, 256 microstep, shared SPI bus |
| RS-485 transceiver | SIT3088EEUA (MSOP-8) | USART2 on PA2/PA3 (no SYSCFG remap needed on G071) |
| 3.3 V logic PSU | TPS54061DRBT (VSON-8-EP) | ~150 mA load (MCU + 2× TMC VCC_IO + RS-485) — see `psu.kicad_sch` |
| Motor supply | VPP — bus V+ or external JPWR | Two SS34 Schottky diodes OR the sources |
| Bus connectors | 2 × JST-XH 4-pin RA | JBUS1, JBUS2 — daisy-chain V+/GND/A/B |
| Motor outputs | 2 × JST-XH 4-pin | JMOT1, JMOT2 — A+ A- B+ B- per bipolar stepper |
| External power | JST-XH 2-pin | JPWR — optional wall-wart for motor supply |
| Sense resistors | 0.1 Ω 1% 1210 × 4 | BRA/BRB pair per TMC, ~1.6 A RMS max |

## Power architecture

```
  JBUS1 pin 1 ─┬── D2 (SS34) ──┐
  JBUS2 pin 1 ─┘                ├── VPP (motor rail, ~5-37 V)
                                 │        │
  JPWR pin 1 ──── D1 (SS34) ─────┘        │
                                          ├── TMC VS (U2, U3) + decoupling
                                          ├── TPS54061 VIN ──► +3V3 logic
                                          │
  (all share GND)

  +3V3 ──► STM32 VDD/VDDA
      ──► TMC VCC_IO (pin 8) on both drivers
      ──► SIT3088E VCC
```

## STM32G071G8U6 pin map

Uses **USART2** on PA2/PA3 (dedicated pins, no SYSCFG remap). The
recommended configuration fits 2 TMC2130s + shared SPI + RS-485 + I²C +
per-axis limit + DIAG monitoring on all 12 free GPIOs, with 1 spare.

| Pin | MCU   | Function         | AF / notes |
|-----|-------|------------------|------------|
| 1   | VDD   | 3V3              | |
| 2   | VDDA  | 3V3 analog       | ferrite + 100 nF recommended |
| 3   | NRST  | Reset            | pull-up + 100 nF |
| 4   | PA0   | **STEP2**        | AF2 TIM2_CH1 — driver 2 pulse |
| 5   | PA1   | RS-485 DE        | GPIO out — tie DE + ~RE together at SIT3088E |
| 6   | PA2   | USART2 TX        | AF1 → SIT3088E DI |
| 7   | PA3   | USART2 RX        | AF1 → SIT3088E RO |
| 8   | PA4   | **TMC1 CS**      | GPIO out |
| 9   | PA5   | SPI1 SCK         | AF0 — shared |
| 10  | PA6   | SPI1 MISO        | AF0 — shared |
| 11  | PA7   | SPI1 MOSI        | AF0 — shared |
| 12  | PA8   | **STEP1**        | AF2 TIM1_CH1 — driver 1 pulse |
| 13  | PA11  | **DRV_ENN** (shared) | GPIO out, active-low. Pulls both TMC DRV_ENN lines simultaneously. |
| 14  | PA12  | **TMC2 CS**      | GPIO out |
| 15  | PA13  | SWDIO            | protected |
| 16  | PA14  | SWCLK            | protected |
| 17  | PA15  | **DIR1**         | GPIO out |
| 18  | PB0   | **Limit 1**      | GPIO in, pull-up (switch to GND) |
| 19  | PB1   | **DIAG1**        | GPIO in + external 47 k pull-up (TMC open-drain) |
| 20  | PB2   | **DIR2**         | GPIO out |
| 21  | PB3   | **Limit 2**      | GPIO in, pull-up |
| 22  | PB4   | **DIAG2**        | GPIO in + external 47 k pull-up |
| 23  | PB5   | Status LED       | GPIO out, 1 kΩ in series |
| 24  | PB6   | I2C1 SCL         | AF6 |
| 25  | PB7   | I2C1 SDA         | AF6 |
| 26  | PB8   | Spare            | — |
| 27  | VSS   | GND              | |
| 28  | VDD_2 | 3V3              | second digital supply pin |

Independent step timers (TIM1 for axis 1, TIM2 for axis 2) allow each
axis to run at its own speed. The shared **DRV_ENN** on PA11 puts both
drivers in/out of enable together — acceptable for most motion-control
use cases where both axes are either live or shut down in sync. If you
need independent enable per axis, reclaim PB8 (the spare) and the design
still fits.

## Sense-resistor assembly variants

Single PCB, two BOM variants. Firmware switches via `CHOPCONF.VSENSE`:

| Variant | R_SENSE | VSENSE bit | Full-scale (CS=31) | Best for |
|---------|---------|-----------|--------------------|----------|
| **Standard** | 0.1 Ω 1% 1210 (LCSC C137091 family) | 0 | ~2.3 A RMS | NEMA17-class steppers, ≥500 mA |
| **Micro** | 0.68 Ω 1% 1210 (LCSC C3000593) | 1 | ~200 mA RMS | 28BYJ-48 (bipolar mode), micro geared steppers, 50–200 mA |

The 0.68 Ω option is placed on the schematic as **DNP** alternates next
to the 0.1 Ω standard parts; for the micro assembly, swap the DNP flag
in the BOM and firmware sets VSENSE=1.

**28BYJ-48 wiring note**: the 28BYJ-48 is a 5-wire unipolar motor. To
drive with the TMC2130 (bipolar only), leave the **red** center-tap
wire disconnected and wire Orange → A+, Yellow → A-, Pink → B+,
Blue → B- at the JMOT connector.

## TMC2130 strapping / required external parts

**This is the checklist to wire in KiCad.** For each TMC2130 (U2, U3):

| Pin | Net / part | Value / note |
|-----|-----------|--------------|
| 8 VCC_IO | 3V3 + 100 nF | logic-level supply |
| 10 SPI_MODE | 3V3 | hardware SPI mode |
| 36 TST_MODE | GND | test mode off |
| 18 DCEN, 19 DCIN | GND | dcStep unused |
| 22 DRV_ENN | MCU GPIO (EN) | software enable |
| 9 DNC, 11 N.C. | GND | unused |
| 12 GNDP, 35 GNDP2, 24 GNDA, 37 EP | GND | multiple ground pins — all tied together |
| 16 VS, 31 VSA | VPP (motor rail) + 10 µF bulk + 100 nF bypass | 4.75–46 V |
| 13–15 OB1/BRB/OB2 | motor coil B, 0.1 Ω BRB to GND | |
| 32–34 OA2/BRA/OA1 | motor coil A, 0.1 Ω BRA to GND | |
| 14 BRB, 33 BRA | 0.1 Ω 1% 1210 sense resistors to GND | + 100 nF ceramic near each |
| 28 CPI, 27 CPO | 22 nF 50V across (charge pump flying cap) | C1532 (basic) |
| 29 VCP | 100 nF 50V to VS | charge pump storage |
| 25 5VOUT | 2.2 µF 6.3V to GNDA | internal 5 V regulator |
| 26 VCC | 470 nF to GND, feed from 5VOUT via 2.2–3.3 Ω | internal digital supply |
| 23 AIN_IREF | 5VOUT through 10 k / 1 k divider, or tie to 5VOUT | current reference |
| 20 DIAG0, 21 DIAG1 | open-drain outputs, 47 k pull-ups to 3V3 | stall/fault |
| 1 CLK | GND | use internal 12 MHz clock |

Sense resistor sizing: `I_rms = V_fs / (R_sense × √2) × (CS+1)/32`.
With default V_fs = 0.325 V, R_sense = 0.1 Ω, CS = 31: **I_rms ≈ 2.3 A**
(exceeds TMC2130's 1.4 A continuous rating — always run with CS reduced
in firmware unless the motor is actively cooled).

## Connectors

| Ref | Connector | Function | Pinout |
|-----|-----------|----------|--------|
| JBUS1, JBUS2 | JST-XH 4-pin RA | RS-485 daisy-chain | 1: V+, 2: B, 3: A, 4: GND |
| JMOT1 | JST-XH 4-pin | Stepper motor 1 | 1: A+, 2: A-, 3: B+, 4: B- |
| JMOT2 | JST-XH 4-pin | Stepper motor 2 | same |
| JPWR | JST-XH 2-pin RA | External motor supply | 1: V+, 2: GND |

## Design rules

Inherited from the servo board's `servo.kicad_pro`:

| Rule | Value |
|------|-------|
| Min track width | 0.15 mm |
| Min clearance | 0.15 mm |
| Min drill | 0.3 mm |
| Min via | 0.45 mm / 0.2 mm drill |
| Stack-up | JLC04161H-7628 (1.6 mm FR4, 4 layers) |
| Board edge clearance | 0.2 mm rule (0.5 mm recommended) |

## Status

**rev 0.1 — skeleton.** All primary ICs, connectors, and power-source
diodes are placed. Per-TMC support circuitry (sense resistors, charge
pump cap, VCP/5VOUT/VCC caps, DIAG pull-ups), STM32 decoupling, SIT3088E
DE wiring, and all nets are **pending** and to be drawn in KiCad. See
the text annotation on the schematic for the per-TMC parts checklist.

## Regeneration

```sh
cd hardware
make docs-ministepper     # schematic SVGs, PCB SVGs, 3D renders, BOM
make jlc-ministepper      # full JLCPCB fab/assembly package
```
