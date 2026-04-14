# Tsumikoro Servo Controller — Hardware

4-layer PCB implementing the Tsumikoro servo/motor controller node. One
STM32G030F6P6 MCU drives four hobby-servo outputs, one DRV8876 H-bridge motor
channel, talks to the RS-485 bus at 1 Mbaud, and exposes an I²C expansion port.

Target manufacturing: **JLCPCB 4-layer (JLC04161H)**, Standard assembly.

![3D top view](images/pcb-3d-top.png)

## Block overview

| Block | Part | Notes |
|-------|------|-------|
| MCU | STM32G030F6P6 (TSSOP-20) | ARM Cortex-M0+ @ 64 MHz, 32 KB flash |
| RS-485 transceiver | SIT3088EEUA (MSOP-8) | 1 Mbaud, half-duplex, on USART1 |
| Motor driver | DRV8876PWPR (HTSSOP-16-EP) | 3.5 A H-bridge, 4.5–37 V, PH/EN mode |
| 3.3 V logic PSU | TPS54061DRBT (VSON-8-EP) | ~50 mA logic load, 60 V Vin capable — see `psu.kicad_sch` |
| 5 V servo PSU | LMR38020SDDAR (SOIC-8-EP) | 2 A synchronous, up to 80 V Vin — see `psu_servo.kicad_sch` |
| ESD protection | SRV05-4 (SOT-23-6) | 4-channel clamp on servo outputs |
| Reverse polarity | SS36 (SMA) | Schottky 3 A / 60 V in series with JPWR |
| Bulk | 220 µF 50 V SMD (Lelon RVT) | Motor-rail bulk near DRV8876 |
| Status | Red LED 0603 | Driven from PA5 |

## Power architecture

```
       JPWR ─┬── D2 (SS36, reverse-polarity) ──┬── VPP (motor rail)
             │                                  ├── DRV8876 VM (pin 11)
             │                                  ├── C12 bulk (220 µF / 50 V)
             │                                  ├── TPS54061 VIN ── psu.kicad_sch ──► +3.3 V logic
             │                                  └── LMR38020  VIN ── psu_servo.kicad_sch ──► +5 V servo
             └── GND (common)

       +3.3 V ──► MCU, SIT3088E, SRV05-4 VCC, I²C pull-ups
       +5 V   ──► JSERVO0..3 Vcc
```

Reverse-polarity protection uses a series Schottky (simple, ~0.5 V drop @ 3 A).
Not an ideal-diode — sufficient for this current class but lossy for larger
builds.

## STM32G030F6P6 pin map (rev 0.1)

See also `CLAUDE.md` at the repo root for the authoritative table.

| Pin | MCU | Function | AF / notes |
|-----|-----|----------|------------|
| 1   | PB7 | DRV8876 nFAULT | GPIO in, 10k pull-up to 3V3 (open-drain) |
| 2   | PB8 | I2C1 SCL | AF6 |
| 3   | PB9 | I2C1 SDA | AF6 |
| 4   | NRST | Reset | — |
| 5   | VDDA | 3V3 analog | — |
| 6   | PA0 | Servo 0 | AF1 TIM3_CH1 |
| 7   | PA1 | RS-485 DE | GPIO out |
| 8   | PA2 | Servo 1 | AF1 TIM3_CH3 |
| 9   | VSS | GND | — |
| 10  | VDD | 3V3 | — |
| 11  | PA3 | Servo 2 | AF1 TIM3_CH4 |
| 12  | PA4 | Motor PH (DIR) | GPIO out → DRV8876 PH |
| 13  | PA5 | Status LED | GPIO out |
| 14  | PA6 | Motor EN (PWM) | AF5 TIM16_CH1, 20 kHz |
| 15  | PA7 | Servo 3 | AF5 TIM17_CH1 |
| 16  | PA8 | Limit switch | GPIO in, pull-up |
| 17  | PA9 (remapped) | USART1 TX | AF1, RS-485 @ 1 Mbaud — **requires SYSCFG PA11_RMP** |
| 18  | PA10 (remapped) | USART1 RX | AF1 — **requires SYSCFG PA12_RMP** |
| 19  | PA13 | SWDIO | protected |
| 20  | PA14 | SWCLK | protected |

## DRV8876 strapping

The DRV8876 is hard-configured for PH/EN mode, no software current control,
always enabled:

| Pin | Net | Rationale |
|-----|-----|-----------|
| PMODE | VCC | PH/EN mode (vs IN/IN) |
| nSLEEP | VCC | Always enabled; no software shutdown |
| IMODE | GND | Current regulation disabled |
| VREF | GND | Unused (no current regulation) |
| IPROPI | NC | Unused |
| nFAULT | 3V3 via 10k → PB7 | Open-drain with pull-up |

Because nSLEEP is tied high, `MOTOR_DIR_COAST` and `MOTOR_DIR_BRAKE` are
hardware-equivalent — EN = 0 gives a synchronous low-side brake. See
`firmware/tsumikoro-servo/src/motor_hbridge.c` for the software behavior.

## Connectors

| Ref | Connector | Function | Pinout |
|-----|-----------|----------|--------|
| JPWR | JST-XH 2-pin RA | Power input (≥6 V) | 1: V+, 2: GND |
| JBUS1/JBUS2 | JST-XH 4-pin RA | RS-485 daisy chain | 1: V+, 2: GND, 3: A, 4: B |
| JSERVO0..3 | JST-PH 3-pin vert | Servo outputs | 1: GND, 2: +5 V, 3: signal |
| JMOTOR2 | JST-PH 2-pin vert | Motor output | 1: OUT1, 2: OUT2 |
| JI2C1 | JST-PH 4-pin vert | I²C expansion | 1: 3V3, 2: GND, 3: SCL, 4: SDA |
| JLIM2 | JST-PH 2-pin vert | Limit switch | 1: PA8 input, 2: GND |
| JP1 | Solder jumper | RS-485 termination (120 Ω in series) | Open by default |

## Board layout

### Schematic — root sheet

![Schematic root](images/servo.svg)

### Schematic — 3.3 V logic PSU (TPS54061)

![PSU sub-sheet](images/servo-PSU.svg)

### Schematic — 5 V servo PSU (LMR38020)

![PSU_SERVO sub-sheet](images/servo-PSU_SERVO.svg)

### PCB — top

![PCB top](images/pcb-top.svg)

### PCB — bottom (mirrored)

![PCB bottom](images/pcb-bottom.svg)

### 3D render — top / bottom

| Top | Bottom |
|-----|--------|
| ![3D top](images/pcb-3d-top.png) | ![3D bottom](images/pcb-3d-bottom.png) |

## BOM

Auto-generated from the schematic. Regenerate with:

```sh
kicad-cli sch export bom \
  --output jlcpcb_bom.csv \
  --fields '${ITEM_NUMBER},Reference,Value,Footprint,${QUANTITY},LCSC,MPN,Manufacturer' \
  --labels 'Item,Designator,Comment,Footprint,Qty,LCSC Part #,MPN,Manufacturer' \
  --group-by 'Value,Footprint' \
  --exclude-dnp \
  servo.kicad_sch
```

Current BOM: [`jlcpcb_bom.csv`](../jlcpcb_bom.csv). Upload directly to JLCPCB
assembly order — column headers already match their expected format.

## Regenerating documentation images

The images in `docs/images/` are produced from the KiCad files via
`kicad-cli`. To refresh them after schematic/PCB changes:

```sh
cd hardware/servo
mkdir -p docs/images

# Schematic sheets (one SVG per sheet)
kicad-cli sch export svg -o docs/images/ servo.kicad_sch

# PCB silkscreen+copper (top and bottom)
kicad-cli pcb export svg -o docs/images/pcb-top.svg \
  --layers "F.Cu,F.Silkscreen,F.Mask,Edge.Cuts" \
  --page-size-mode 2 servo.kicad_pcb

kicad-cli pcb export svg -o docs/images/pcb-bottom.svg \
  --layers "B.Cu,B.Silkscreen,B.Mask,Edge.Cuts" \
  --mirror --page-size-mode 2 servo.kicad_pcb

# 3D renders
kicad-cli pcb render --side top    --width 1600 --height 1000 \
  -o docs/images/pcb-3d-top.png    servo.kicad_pcb
kicad-cli pcb render --side bottom --width 1600 --height 1000 \
  -o docs/images/pcb-3d-bottom.png servo.kicad_pcb
```

## Design rules

Project set to JLCPCB 4-layer standard limits (currently running conservative
values for yield):

| Rule | Value |
|------|-------|
| Min track width | 0.15 mm (JLCPCB 4-layer min is 0.09) |
| Min clearance | 0.15 mm (JLCPCB min 0.09) |
| Min drill | 0.3 mm (JLCPCB min 0.15) |
| Min via | 0.45 mm dia / 0.2 mm drill |
| Board edge clearance | 0.2 mm rule (0.5 mm recommended) |
| Stack-up | JLC04161H-7628 (1.6 mm FR4, 4 layers) |

The DRC report (`kicad-cli pcb drc`) shows no clearance or routing violations
at the time of writing.

## Known limitations / future work

- Reverse-polarity via series Schottky loses ~0.5 V / 1.5 W at 3 A — consider
  an ideal-diode FET for higher-current future variants.
- The 220 µF aluminum electrolytic bulk cap is the tallest component on the
  board (~10 mm). If profile matters, switch to polymer or multiple ceramics.
- PCB only uses front-side placement; adding back-side assembly would allow a
  more compact layout but requires Standard assembly tier at JLCPCB.
- Board addressing lives in the last 2 KB flash sector (`0x08007800`); each
  unit needs to be provisioned at manufacturing time (see CLAUDE.md).
