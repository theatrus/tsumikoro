# Tsumikoro ESP32 RS-485 Bridge — Hardware

4-layer PCB acting as a bus-powered gateway between the Tsumikoro RS-485
node bus and a user-supplied ESP32-DevKitC-V4 module. No local power input
— the board draws from either JBUS1 or JBUS2 V+ rail, regulates to 3.3 V
via an LMR38020 synchronous buck, and exposes one RS-485 transceiver to
the ESP32 over UART.

Target manufacturing: **JLCPCB 4-layer (JLC04161H)**, Standard assembly
(includes through-hole connectors + ESP32 headers).

![3D top view](images/pcb-3d-top.png)

## Block overview

| Block | Part | Notes |
|-------|------|-------|
| MCU | **ESP32-DevKitC-V4** (user-supplied module) | Plugs into two 1×19 female headers at 1" row spacing |
| RS-485 transceiver | SIT3088EEUA (MSOP-8) | Same part as servo board; connects to an ESP32 UART |
| 3.3 V PSU | LMR38020SDDAR (SOIC-8-EP) | 3.8–80 V in, 2 A out — see `psu.kicad_sch` |
| Inductor | FNR4030S 10 µH, 4×4 mm | PSU buck inductor |
| Bus connectors | 2 × JST-XH 4-pin RA | JBUS1, JBUS2 — daisy-chain to servo nodes |
| Power source | Bus V+ only (no local JPWR) | Fed by any upstream node on JBUS1 or JBUS2 |

## Power architecture

```
  JBUS1 pin 1 ──┐
                ├── Bus V+ (5–37 V from upstream node)
  JBUS2 pin 1 ──┘       │
                        ├── LMR38020 VIN ──► psu.kicad_sch ──► +3.3 V
                        └── (optional TVS / bulk)

  +3.3 V ──► ESP32 3V3 pin
          ──► SIT3088E VCC
```

The bridge is **downstream-only** on the bus in terms of power — it takes
whatever V+ the bus carries and regulates it locally. There is no
reverse-polarity Schottky because bus power is assumed clean and
polarity-guaranteed by JST-XH keying. If you want the option to power the
entire bus *from* the bridge (e.g. plug it into wall-warts via an
inline injector), you would add a Schottky from a local supply into bus
V+ — not part of this rev.

The LMR38020 feedback divider is retuned for 3.3 V:

- Vref (typ): 1.0 V
- R2 (top) = **22 kΩ 0402 (basic, C25768)**
- R3 (bottom) = **10 kΩ 0402 (basic, C25744)**
- Vout = 1.0 × (1 + 22 / 10) = **3.20 V** (within ESP32 tolerance 3.0–3.6 V)

## ESP32-DevKitC-V4 pinout reference

The board accepts a standard ESP32-DevKitC-V4 (38-pin) module on two 1×19
female headers (`J_ESP_L`, `J_ESP_R`) spaced 1" (25.4 mm) apart — same as
the DevKitC reference carrier.

Planned signal assignment (to be wired in the schematic):

| Header pin (left row, GPIO side) | ESP32 pin | Wire to |
|---|---|---|
| 3V3 (pin 1) | +3V3 out from DevKit's own regulator | **leave NC** (we power DevKit via VIN) |
| EN | EN (reset) | optional: user button via pull-up |
| GPIO 17 (UART2 TX) | U2TXD | SIT3088E DI |
| GPIO 16 (UART2 RX) | U2RXD | SIT3088E RO |
| GPIO 4 (any free) | — | SIT3088E DE + ~RE (tied together) |
| GND | GND | GND |

| Header pin (right row, USB/power side) | ESP32 pin | Wire to |
|---|---|---|
| VIN (pin 1) | 5 V in | +3.3 V out from our LMR38020, or leave NC if the ESP32 must see 5 V — see note |
| GND | GND | GND |
| Other GPIOs | — | Broken out to a header for expansion (optional) |

**Note on VIN vs 3V3**: the ESP32-DevKitC-V4 has its own AMS1117 LDO on
the VIN rail expecting ~5 V. If we feed our regulated 3.3 V directly into
VIN, the module's internal LDO will drop to ~2.5 V — under-voltage for
the ESP32. **Options**:

1. **Wire our 3.3 V to the DevKit's 3V3 pin directly** (bypass the onboard LDO) — simplest, works with most clone boards.
2. **Retune our PSU to 5 V** and feed the VIN pin — lets the DevKit's LDO do its job. Needs R2 = 40.2 kΩ instead of 22 kΩ (both pads fit the 0402 footprint).

The current rev targets option 1 (3.3 V direct to the 3V3 pin). If you
choose option 2, swap R2 in `psu.kicad_sch` and rewire the header
connection.

## Connectors

| Ref | Connector | Function | Pinout |
|-----|-----------|----------|--------|
| JBUS1, JBUS2 | JST-XH 4-pin RA | Daisy-chained RS-485 bus | 1: bus V+, 2: B, 3: A, 4: GND |
| J_ESP_L | 1×19 female pin socket, 2.54 mm | ESP32-DevKitC-V4 left row | see module datasheet |
| J_ESP_R | 1×19 female pin socket, 2.54 mm | ESP32-DevKitC-V4 right row | see module datasheet |

Row-to-row spacing between J_ESP_L and J_ESP_R: **25.4 mm (1 inch)** —
matches the ESP32-DevKitC-V4 body.

## Board layout

### Schematic — root sheet

![Schematic root](images/bridge.svg)

### Schematic — PSU sub-sheet (LMR38020 @ 3.3V)

![PSU sub-sheet](images/bridge-PSU.svg)

### PCB — top / bottom

| Top | Bottom (mirrored) |
|-----|-------------------|
| ![top](images/pcb-top.svg) | ![bottom](images/pcb-bottom.svg) |

| 3D top | 3D bottom |
|--------|-----------|
| ![3D top](images/pcb-3d-top.png) | ![3D bottom](images/pcb-3d-bottom.png) |

## BOM

Auto-generated. Regenerate with:

```sh
cd hardware
make bom-bridge     # just the BOM
make docs-bridge    # images + BOM
```

Current BOM: [`jlcpcb_bom.csv`](../jlcpcb_bom.csv).

## Manufacturing package

```sh
cd hardware
make jlc-bridge
```

Produces `bridge-jlcpcb.zip` plus the unpacked `bridge/jlcpcb/` directory
with gerbers, merged drill, JLCPCB-format BOM, and CPL. Upload the zip to
<https://cart.jlcpcb.com/quote>.

## Design rules

Same as servo board — 4-layer JLC04161H-7628 stackup inherited from
`servo.kicad_pro`:

| Rule | Value |
|------|-------|
| Min track width | 0.15 mm |
| Min clearance | 0.15 mm |
| Min drill | 0.3 mm |
| Min via | 0.45 mm dia / 0.2 mm drill |
| Board edge clearance | 0.2 mm rule (0.5 mm recommended) |
| Stack-up | JLC04161H-7628 (1.6 mm FR4, 4 layers) |

## Status

Rev 0.1 — schematic skeleton with all ICs/connectors/passives placed, but
**wiring is pending**. The LMR38020 pinout is an informed guess and
should be verified against TI's SLUSDS8 datasheet before the first fab
run. Board outline and layout haven't been drawn yet.
