# Hardware TODO

## servo (rev 0.1)

- [x] Schematic: all ICs, passives, connectors, LEDs placed + wired
- [x] PCB layout routed, 59×43mm, 4-layer JLC04161H
- [x] Rail-selectable servo power (JP2/JP3/JP4 — 5V/VPP)
- [x] D2/D3 reverse-polarity + bus isolation
- [x] JPTERM termination jumper
- [x] Docs + auto-generated BOM + JLCPCB package
- [ ] Prototype build & validation against the test plan in PR #46

## bridge (rev 0.1)

- [x] Schematic components placed: 2 × SIT3088E, 2 × JBUS, 2 × DC jack, ESP32-S3-MINI-1-N8, USB-C, buttons, strapping, terminations
- [x] PSU sub-sheet: LMR38020 @ 3.2V
- [x] Vendored Espressif S3-MINI/DevKitC library + 3D STEPs under CC-BY-SA
- [x] Vendored TS-1187A footprint + real STEP
- [x] SMT 16P USB-C footprint (GCT USB4105) with 3D model
- [ ] **Net/wire routing** in the schematic (all components placed, no connections drawn)
- [ ] PCB layout & routing
- [ ] Firmware for the ESP32-S3 bridge
- [ ] Prototype build

## ministepper (rev 0.1) — TMC2130 dual stepper driver

Target MCU: **STM32G071G8U6** (UFQFPN-28) — matches existing `firmware/tsumikoro-ministepper/` target.

- [ ] Project structure: `bridge.kicad_pro` → copy layout/stackup from servo
- [ ] Root schematic with 2 × TMC2130, STM32G071, SIT3088E, 2 × JBUS, 2 × stepper output, optional external motor supply
- [ ] Reuse TPS54061 PSU sub-sheet (3.3V for logic)
- [ ] Motor supply select: bus V+ or external via a DC jack / XH connector, protected with Schottky
- [ ] Per-TMC support: sense resistors (2 × 0.1Ω 1210), charge pump cap 22nF, VCP cap 100nF, 5VOUT cap 2.2µF, VCC cap, DIAG pull-ups
- [ ] STM32 pin assignments (below — open to revision once firmware is written)
- [ ] PCB layout
- [ ] Docs + BOM + JLCPCB package
- [ ] Firmware pin map update in `firmware/tsumikoro-ministepper/` + `CLAUDE.md`

### Tentative pin map (STM32G071G8U6, UFQFPN-28)

Using **USART2** on PA2/PA3 for RS-485 (dedicated pins, no SYSCFG remap
needed — avoids the PA11/PA12 shared-pad issue the servo hit on G030).

| Pin | MCU | Function | Notes |
|-----|-----|----------|-------|
| 1 | VDD | 3V3 | |
| 2 | VDDA | 3V3 analog | |
| 3 | NRST | Reset | |
| 4 | PA0 | TMC1 CS | SPI1 NSS alt |
| 5 | PA1 | RS-485 DE | GPIO out |
| 6 | PA2 | USART2 TX | AF1 — RS-485 |
| 7 | PA3 | USART2 RX | AF1 |
| 8 | PA4 | TMC2 CS | GPIO out |
| 9 | PA5 | SPI1 SCK | AF0 — shared by both TMCs |
| 10 | PA6 | SPI1 MISO | AF0 |
| 11 | PA7 | SPI1 MOSI | AF0 |
| 12 | PA8 | TMC1 STEP | timer PWM for high-rate step (TIM1_CH1) |
| 13 | PA11 | TMC2 STEP | TIM1_CH4 |
| 14 | PA12 | Motor DIR mux | or individual DIR; TBD |
| 15 | PA13 | SWDIO | |
| 16 | PA14 | SWCLK | |
| 17 | PA15 | TMC1 DIR | |
| 18 | PB0 | TMC1 EN | |
| 19 | PB1 | TMC1 DIAG1 | stallguard interrupt |
| 20 | PB2 | TMC2 DIR | |
| 21 | PB3 | TMC2 EN | |
| 22 | PB4 | TMC2 DIAG1 | |
| 23 | PB5 | Status LED | |
| 24 | PB6 | Limit 1 | optional |
| 25 | PB7 | Limit 2 | optional |
| 26 | PB8 | Spare | I²C or future expansion |
| 27 | VSS | GND | |
| 28 | VDD_2 | 3V3 | |

## light-sensor (rev 0.1) — I²C RGB + illuminator

- [ ] Research: pick RGB sensor IC (candidates: TCS34725, VEML6040, APDS-9960, AS7341)
- [ ] Research: illuminator — white LED + FET driver (PWM brightness) vs fixed-current LED
- [ ] Research (optional): IR presence sensor — PIR or VL53L0X / VL53L1X ToF
- [ ] Small MCU or host via I²C? Decide: straight I²C peripheral board (no local MCU) vs STM32G030 with I²C/RS-485 bridge like the servo
- [ ] Project structure: `light-sensor.kicad_pro` etc.
- [ ] Schematic, PCB, docs
- [ ] Same PSU design (TPS54061 3V3) if bus-powered; else AMS1117 cascade
- [ ] Firmware (if local MCU)

## Shared / infrastructure

- [ ] Run full KiCad DRC (clearance/silk) inside the PCB editor on each board before fab
- [ ] Add a `make drc` Makefile target wrapping `kicad-cli pcb drc`
- [ ] Consider adding `make erc` wrapping `kicad-cli sch erc`
- [ ] Verify PCB rotation offsets for JLCPCB assembly on first prototype run
- [ ] Add ESP32-S3 firmware skeleton under `firmware/tsumikoro-bridge/` (or keep ESPHome config)
