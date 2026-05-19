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

Target MCU: **STM32G071G8U6** (UFQFPN-28) — matches `firmware/tsumikoro-ministepper/`.

- [x] Project structure and 4-layer JLC04161H stackup (copy of servo)
- [x] Root schematic with 2 × TMC2130, STM32G071, SIT3088E, 2 × JBUS,
      2 × stepper output, external motor supply + Schottky OR-gating
- [x] TPS54061 PSU sub-sheet (3.3V for logic) — inherited from servo
- [x] Final pin map (see CLAUDE.md and `docs/README.md`); selected
      recommended config (2 drivers + shared EN + per-axis limit + DIAG
      + I²C + RS-485 + LED, 1 spare GPIO)
- [x] Firmware `main.c` updated: USART2 on PA2/PA3 (no remap dance),
      all PIN_* defines matched to schematic, LED moved to PB5
- [ ] **Nets/wiring in the schematic** (placed but not yet connected)
- [ ] Per-TMC support placements: 2 × 0.1 Ω 1210 sense resistors
      (standard variant) + 2 × 0.68 Ω 1210 DNP alternates (micro /
      28BYJ-48 variant). Plus 22 nF charge pump cap, 100 nF VCP,
      2.2 µF 5VOUT, 470 nF VCC, 47 k DIAG pull-ups × 2 drivers.
- [ ] STM32 decoupling on each VDD rail (100 nF + bulk 10 µF)
- [ ] PCB layout
- [ ] Firmware: actual SPI driver for TMC2130 register writes
- [ ] Firmware: TIM1_CH1 / TIM2_CH1 step-pulse generation (one-shot or
      DMA-driven variable-rate)
- [ ] Firmware: tsumikoro bus command handlers for per-axis move/status

### Pin map: see `CLAUDE.md > Ministepper PCB Pin Map` (authoritative)
and `hardware/ministepper/docs/README.md`. Firmware `PIN_*` defines in
`firmware/tsumikoro-ministepper/src/main.c` are the tie-breaker if docs
drift — rule of thumb: update both or neither.

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
