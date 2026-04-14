# Tsumikoro ESP32 RS-485 Bridge — Hardware

4-layer PCB acting as a dual-bus gateway between two independent
Tsumikoro RS-485 segments and a user-supplied ESP32-DevKitC-V4 module.
Each bus has its own SIT3088E transceiver, its own JST-XH connector,
its own DC barrel jack for direct power injection (with Schottky
reverse-polarity protection), and its own solder-jumper-enabled 120 Ω
end-of-line termination. Board regulates incoming V+ to 3.3 V via an
LMR38020 synchronous buck for the ESP32.

Target manufacturing: **JLCPCB 4-layer (JLC04161H)**, Standard assembly
(includes through-hole connectors + ESP32 headers).

![3D top view](images/pcb-3d-top.png)

## Block overview

| Block | Part | Notes |
|-------|------|-------|
| MCU | **ESP32-DevKitC-V4** (user-supplied module) | Plugs into two 1×19 female headers at 1" row spacing |
| RS-485 transceivers | 2 × SIT3088EEUA (MSOP-8) | U1 for Bus 1, U3 for Bus 2 — each on its own ESP32 UART |
| Terminations | 2 × 120 Ω 1206 + solder jumper | `R4`/`JPTERM1` for Bus 1, `R5`/`JPTERM2` for Bus 2 — close the jumper at each bus endpoint |
| 3.3 V PSU | LMR38020SDDAR (SOIC-8-EP) | 3.8–80 V in, 2 A out — see `psu.kicad_sch` |
| Inductor | FNR4030S 10 µH, 4×4 mm | PSU buck inductor |
| Bus connectors | 2 × JST-XH 4-pin RA | JBUS1, JBUS2 — each serves a **separate** RS-485 segment |
| DC power jacks | 2 × DC-005K-5A-2.0 (C2880546) | JDC1, JDC2 — optional wall-wart input per bus |
| Reverse polarity | 2 × MBRD10200 (DPAK, 10 A/200 V) | D1 in line with JDC1→Bus 1 V+; D2 for Bus 2 |
| Power source | Either/both JBUS V+ rails **or** either/both DC jacks | Diodes OR the sources per bus |

## Power architecture

```
  JDC1 (DC barrel) ── D1 (MBRD10200) ──┐
                                        ├── JBUS1 V+ (Bus 1 supply rail)
  JBUS1 pin 1 ──────────────────────────┘
                                               │
                                               ├── LMR38020 VIN (via wire-OR with JBUS2 V+)
                                               │                     │
  JDC2 (DC barrel) ── D2 (MBRD10200) ──┐       │                     ▼
                                        ├── JBUS2 V+ (Bus 2 supply rail)
  JBUS2 pin 1 ──────────────────────────┘       │
                                               │      ┌──► +3V3 to ESP32 3V3 pin
                                               └──────┤
                                                      └──► SIT3088E VCC for U1, U3

  Bus data (per bus):
      JBUS1 A/B ──► U1 (SIT3088E) ──► ESP32 UART1
      JBUS2 A/B ──► U3 (SIT3088E) ──► ESP32 UART2
      R4 + JPTERM1 = 120 Ω series-jumper termination at Bus 1 endpoint
      R5 + JPTERM2 = 120 Ω series-jumper termination at Bus 2 endpoint
```

Each bus is **electrically independent** — they share only the ESP32, the
3.3 V rail, and GND. This lets the ESP32 act as a protocol gateway, a
packet switch, or bridge two different bus speeds/topologies. Power can
be injected on either bus (via DC jack or upstream bus connector) and
the LMR38020 pulls whichever rail is highest. The Schottkys prevent
one bus from back-feeding the other or the DC jacks from back-feeding
each other.

The LMR38020 feedback divider is retuned for 3.3 V:

- Vref (typ): 1.0 V
- R2 (top) = **22 kΩ 0402 (basic, C25768)**
- R3 (bottom) = **10 kΩ 0402 (basic, C25744)**
- Vout = 1.0 × (1 + 22 / 10) = **3.20 V** (within ESP32 tolerance 3.0–3.6 V)

## Recommended ESP32 pinout

Two independent RS-485 buses need two UARTs plus a DE/~RE control line
each. The ESP32 has three hardware UARTs; UART0 is used for USB serial
(programming and log output), leaving UART1 and UART2 free. UART1's
default pins (GPIO 9/10) land on the SPI flash pads of most ESP32
modules and **must not** be used — they have to be remapped via the
GPIO matrix.

| Signal | ESP32 GPIO | Wire to | Notes |
|--------|-----------|---------|-------|
| **Bus 1 — UART2 (default pins)** |  |  |  |
| U2 TX | GPIO 17 | U1 pin 4 (DI) | Default UART2 TX |
| U2 RX | GPIO 16 | U1 pin 1 (RO) | Default UART2 RX |
| Bus 1 DE/~RE | GPIO 4 | U1 pin 3 (DE) + pin 2 (~RE) | Tie DE+~RE together; high = drive, low = receive |
| **Bus 2 — UART1 (remapped)** |  |  |  |
| U1 TX | GPIO 25 | U3 pin 4 (DI) | Any free GPIO works — remap via `uart_set_pin()` |
| U1 RX | GPIO 26 | U3 pin 1 (RO) |  |
| Bus 2 DE/~RE | GPIO 27 | U3 pin 3 (DE) + pin 2 (~RE) |  |
| **Power** |  |  |  |
| 3V3 pin (J_ESP_L pin 1) | 3V3 | +3V3 from LMR38020 | **Bypass the DevKit's onboard AMS1117** by feeding 3V3 directly — see caveat below |
| GND | GND | GND | Shared with everything |
| EN | — | pull-up + optional reset button | Leave NC for now; break out later |

**Avoid on ESP32 (don't route GPIOs to these):**

- **GPIO 0, 2, 12, 15** — boot strapping pins (affect flash/boot mode)
- **GPIO 1, 3** — UART0 (USB serial programming)
- **GPIO 5, 18, 19, 23** — default VSPI (safe as GPIO if you're not using SPI peripherals, but ESP-IDF libraries may claim them)
- **GPIO 6–11** — SPI flash, physically inaccessible on most DevKitC boards
- **GPIO 34–39** — input-only pins (no TX, no DE output)

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
| JBUS1 | JST-XH 4-pin RA | **Bus 1** — RS-485 segment A | 1: V+, 2: B, 3: A, 4: GND |
| JBUS2 | JST-XH 4-pin RA | **Bus 2** — RS-485 segment B (electrically separate) | 1: V+, 2: B, 3: A, 4: GND |
| JDC1 | DC barrel (2.0 mm center) | Bus 1 power input | Tip: V+, Sleeve: GND, Switch: NC |
| JDC2 | DC barrel (2.0 mm center) | Bus 2 power input | Tip: V+, Sleeve: GND, Switch: NC |
| J_ESP_L | 1×19 female pin socket, 2.54 mm | ESP32-DevKitC-V4 left row | see module datasheet |
| J_ESP_R | 1×19 female pin socket, 2.54 mm | ESP32-DevKitC-V4 right row | see module datasheet |
| JPTERM1 | 2-pin solder jumper | Bus 1 120 Ω termination enable (series with R4) | Open by default; close at endpoint |
| JPTERM2 | 2-pin solder jumper | Bus 2 120 Ω termination enable (series with R5) | Open by default; close at endpoint |

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
