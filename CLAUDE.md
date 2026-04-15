# Claude Code Assistant Guidelines

## General Rules
- Do not run cat or stty commands
- Do not read debug messages from the serial port - ask the user to do that
- Use makefile targets when possible
- Always reset the device after flashing: `st-flash write <file> 0x08000000 --reset`

## Build Commands

### STM32 Firmware (NUCLEO-G071RB)
```bash
cd firmware/build
make                    # Build all targets
make clean             # Clean build artifacts
```

Available targets:
- `nucleo-g071rb/nucleo-g071rb.elf` - Bare-metal firmware
- `nucleo-g071rb-freertos/nucleo-g071rb-freertos.elf` - FreeRTOS firmware

### ESP32 Firmware (tsumikoro-bridge)
```bash
cd firmware/tsumikoro-bridge
make build             # Build ESPHome firmware
make clean             # Clean build
make upload            # Upload to ESP32
```

## Debug Flags

### STM32 HAL Debug (tsumikoro_hal_stm32.c)
Enable in CMakeLists.txt:
```cmake
target_compile_definitions(tsumikoro_bus PRIVATE TSUMIKORO_HAL_DEBUG=1)
```

Enables debug output:
- `[HAL_INIT]` - Initialization steps
- `[HAL_RX]` - RX data with hex dumps
- `[HAL_TX]` - TX data with hex dumps
- `[DMA UPDATE]` - DMA position updates
- `[RTO]` - Receiver timeout events
- `[HAL]` - Callback invocations

### Bus Protocol Debug (tsumikoro_bus.c)
Enable in CMakeLists.txt:
```cmake
target_compile_definitions(tsumikoro_bus PRIVATE TSUMIKORO_BUS_DEBUG=1)
```

Enables debug output:
- `[BUS]` - State machine transitions
- `[BUS RX CB]` - RX callback and queue operations
- `[BUS] RX thread` - RX thread operations
- `[BUS] TX thread` - TX thread operations
- `[BUS] Handler` - Handler thread operations
- Packet decode/encode details

### STM32 Interrupt Counters
The firmware tracks interrupt counts:
- `g_uart1_irq_count` - UART1 interrupt count
- `g_dma_rx_irq_count` - DMA RX interrupt count
- `g_dma_tx_irq_count` - DMA TX interrupt count

These are displayed in the Stats task output every 10 seconds.

## Flashing STM32 Firmware

### Using Makefile Targets (Recommended)
```bash
cd firmware
make flash-nucleo-g071rb-freertos   # Flash FreeRTOS firmware
make flash-nucleo-g071rb            # Flash bare-metal firmware
make flash-servo-g030               # Flash servo (STM32G030)
make flash-servo-g071               # Flash servo (STM32G071)
make flash-ministepper-g071         # Flash ministepper
make reset                          # Reset STM32 target
```

### Using st-flash Directly
```bash
# Always use --reset flag before write command
st-flash --reset write build/nucleo-g071rb-freertos/nucleo-g071rb-freertos/nucleo-g071rb-freertos.bin 0x08000000
st-flash --reset write build/nucleo-g071rb/nucleo-g071rb/nucleo-g071rb.bin 0x08000000
st-flash reset  # Reset only
```

The `--reset` flag ensures the device properly resets after flashing.

## Serial Console

The NUCLEO board has two serial ports:
- **USART2** (ST-Link VCP): Debug output at 115200 baud - PA2(TX), PA3(RX)
- **USART1** (Bus protocol): 1 Mbaud RS-485 - PC4(TX), PC5(RX), PA1(DE)

User should monitor USART2 for debug output using their preferred serial tool.

## Servo PCB Pin Map (STM32G030F6P6 TSSOP-20, rev 0.1)

Custom PCB (`hardware/servo/`) using the STM32G030F6P6 as the servo/motor
controller. Pin count is tight; every GPIO is assigned.

| Pin | MCU         | Function       | AF / notes                                 |
**rev 0.1 of this board is NOT TO BE FABBED.** The original KiCad
symbol had a fabricated pinout that didn't match ST DS12991 Figure 4 —
VDD/VDDA are fused on pad 4 (not split on pads 4/5), VSS is pad 5 (not
pad 9), NRST is pad 6 (not pad 4), and every GPIO was shifted. The
servo wiring followed the fake symbol, so the board as drawn would
short +3V3 to GND through pad 5 and never power the MCU. See the big
`(text)` block on the root schematic for rework notes.

**Corrected pin map (rev 0.2 target, per ST DS12991 Figure 4 / Table 12).**
Note: pads 1, 2, 15, 16, 17, 19, 20 each bond multiple die pads in
parallel — firmware picks which die signal to drive by enabling the
corresponding GPIO (only one at a time per pad).

|Pin | MCU (selected)  | Function          | Notes |
|----|-----------------|-------------------|-------|
| 1  | PB8 (of PB7/PB8)| I2C1 SCL          | AF6. PB7 not used. |
| 2  | PB9 (of PB9/PC14)| I2C1 SDA         | AF6. |
| 3  | PC15            | Spare             | — |
| 4  | VDD/VDDA        | +3V3              | Single pin on this package. Bulk + 100 nF at pin. |
| 5  | VSS/VSSA        | GND               | |
| 6  | NRST            | Reset             | 100 nF to GND; internal pull-up. |
| 7  | PA0             | Servo 0           | AF1 TIM3_CH1 |
| 8  | PA1             | RS-485 DE         | GPIO out |
| 9  | PA2             | Servo 1           | AF1 TIM3_CH3 |
| 10 | PA3             | Servo 2           | AF1 TIM3_CH4 |
| 11 | PA4             | Motor PH (DIR)    | GPIO → DRV8876 PH |
| 12 | PA5             | Status LED        | GPIO out, 1k in series |
| 13 | PA6             | Motor EN (PWM)    | AF5 TIM16_CH1, 20 kHz → DRV8876 EN |
| 14 | PA7             | Servo 3           | AF5 TIM17_CH1 |
| 15 | PA8 (of PA8/PB0/PB1/PB2) | Limit switch | GPIO in, internal pull-up |
| 16 | PA9 (PA11→PA9 remap) | USART1 TX    | AF1 |
| 17 | PA10 (PA12→PA10 remap) | USART1 RX  | AF1 |
| 18 | PA13            | SWDIO             | |
| 19 | PA14-BOOT0 (of PA15/PA14) | SWCLK + BOOT0 | 10k pull-down + TP_BOOT0 |
| 20 | PB3/PB4/PB5/PB6 | Spare             | — |

**DROPPED from rev 0.1 design:** `PB7 DRV8876 nFAULT` — pad 1 carries
PB7 and PB8 on the same bond, so keeping nFAULT would block I²C SCL.
nFAULT monitoring becomes firmware-indirect (current-sense or timeout)
instead of a dedicated GPIO.

**SYSCFG remaps required at init** (BEFORE configuring AF on those pads):
- `SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP`
  to route PA9/PA10 (USART1) to pads 16/17.

**DRV8876 strapping on this board (PH/EN mode):**
- `PMODE` → VCC (PH/EN mode)
- `nSLEEP` → VCC (always enabled; no software shutdown)
- `IMODE` → GND (current regulation disabled)
- `VREF` → GND (unused)
- `IPROPI` → no connect
- `nFAULT` → tied off (not monitored — was on PB7 in rev 0.1, removed in rev 0.2)

Because `nSLEEP` is hard-tied high, `MOTOR_DIR_COAST` and `MOTOR_DIR_BRAKE`
are hardware-equivalent — EN=0 gives a synchronous low-side brake. This
is documented in `inc/motor_hbridge.h`.

**Hardware ID / addressing:** There are no HW-ID strap pins. Per-unit
addressing lives in the last 2 KB flash config sector
(0x08007800–0x08007FFF), provisioned at manufacturing time. Any
`Read_Hardware_ID()` using PB7/PB8 reads is legacy and dead.

## Ministepper PCB Pin Map (STM32G071G8U6 UFQFPN-28, rev 0.1)

Custom PCB (`hardware/ministepper/`). Dual TMC2130 stepper driver node.
Uses **USART2 on PA2/PA3** — dedicated pins, no SYSCFG remap needed
(unlike the servo's G030 where PA11/PA12 shared pads with PA9/PA10).

Authoritative pinout is ST DS12232 Figure 9 (STM32G071GxU UFQFPN28).
On this package, VDD and VDDA are **internally merged** on pin 3 —
there is no separate VDDA pin, no VDD_2, and PB2 is not bonded out.

| Pin | MCU         | Function           | AF / notes |
|-----|-------------|--------------------|------------|
| 1   | PC14/OSC32_IN | Spare            | (LSE input if later needed; GPIO otherwise) |
| 2   | PC15/OSC32_OUT| Spare            | (LSE output if later needed; GPIO otherwise) |
| 3   | VDD/VDDA    | 3V3 (merged)       | 100 nF + bulk close to pin; optional ferrite on VDDA path |
| 4   | VSS/VSSA    | GND                | — |
| 5   | PF2/NRST    | Reset              | 10 k pull-up (internal) + 100 nF to GND |
| 6   | PA0         | STEP2              | AF2 TIM2_CH1 — driver 2 step pulse |
| 7   | PA1         | RS-485 DE          | GPIO output |
| 8   | PA2         | USART2 TX          | AF1 — RS-485 |
| 9   | PA3         | USART2 RX          | AF1 |
| 10  | PA4         | TMC1 CS            | GPIO output (manual SPI CS) |
| 11  | PA5         | SPI1 SCK           | AF0 — shared by both TMCs |
| 12  | PA6         | SPI1 MISO          | AF0 |
| 13  | PA7         | SPI1 MOSI          | AF0 |
| 14  | PB0         | Limit 1            | GPIO input, internal pull-up (switch to GND) |
| 15  | PB1         | DIAG1              | GPIO input + 47k pull-up (open-drain from TMC) |
| 16  | PA8         | STEP1              | AF2 TIM1_CH1 — driver 1 step pulse |
| 17  | **PC6**     | **DIR2**           | GPIO output — replaces the non-existent PB2 |
| 18  | PA11 [PA9]  | DRV_ENN (shared)   | GPIO out, active-low enable for both TMCs |
| 19  | PA12 [PA10] | TMC2 CS            | GPIO output |
| 20  | PA13        | SWDIO              | protected |
| 21  | PA14-BOOT0  | SWCLK + BOOT0      | 10 k pull-down + test pad; see "BOOT0 handling" |
| 22  | PA15        | DIR1               | GPIO output |
| 23  | PB3         | Limit 2            | GPIO input, internal pull-up |
| 24  | PB4         | DIAG2              | GPIO input + 47k pull-up |
| 25  | PB5         | Status LED         | GPIO output |
| 26  | PB6         | I2C1 SCL           | AF6 |
| 27  | PB7         | I2C1 SDA           | AF6 |
| 28  | PB8         | Spare              | — |
| EP  | VSS         | GND (exposed pad)  | via stitching to inner GND plane |

STEP1 and STEP2 use **independent timers** (TIM1 vs TIM2) so each axis
can run at its own rate. CS lines are software-managed GPIOs — the SPI
peripheral is shared across both TMC2130s, so firmware asserts the
target CS low before each transaction.

**Sense-resistor variants** (assembly-time choice, same PCB):
- **Standard**: 0.1 Ω 1% 1210, `CHOPCONF.VSENSE = 0` → up to ~1.5 A RMS
  per phase for NEMA17-class steppers.
- **Micro**: 0.68 Ω 1% 1210, `CHOPCONF.VSENSE = 1` → ~50–200 mA range
  for small geared steppers including 28BYJ-48 run in bipolar mode
  (disconnect the red center tap wire).

Keep the `PIN_*` defines in `firmware/tsumikoro-ministepper/src/main.c`
in sync with this table. They are the source of truth for firmware pin
assignments and must match the schematic in `hardware/ministepper/`.

## GDB Debugging

### Standard debugging:
```bash
# Terminal 1:
st-util

# Terminal 2:
cd firmware/build
arm-none-eabi-gdb nucleo-g071rb-freertos/nucleo-g071rb-freertos.elf
(gdb) target extended-remote :4242
(gdb) load
(gdb) monitor reset
(gdb) continue
```

### RX data flow debugging:
```bash
# Terminal 1:
st-util

# Terminal 2:
cd firmware/build
arm-none-eabi-gdb -x .gdbinit-rx-debug
```

The `.gdbinit-rx-debug` file sets up breakpoints for RX flow:
1. UART IRQ handler (receiver timeout)
2. HAL RX callback
3. RX thread
4. Packet decode
5. Handler thread

## Memory Usage

### NUCLEO-G071RB (STM32G071RBT6)
- Flash: 128 KB
- RAM: 36 KB

### FreeRTOS Memory Allocation
- Heap: 18 KB (configTOTAL_HEAP_SIZE)
- Bus threads: 2048/2048/3072 bytes (RX/TX/Handler)
- App tasks: 512/512/1536 bytes (LED/Button/Stats)
- Free heap during operation: ~3-5 KB

Stack usage monitoring via `uxTaskGetStackHighWaterMark()` is displayed every 10s in Stats task.

## Common Workflows

### Build and flash:
```bash
cd firmware
make nucleo-g071rb-freertos flash-nucleo-g071rb-freertos
```

### Build all and flash specific target:
```bash
cd firmware
make nucleo-g071rb-freertos   # Build
make flash-nucleo-g071rb-freertos  # Flash
```

### Enable debug and rebuild:
1. Edit `firmware/nucleo-g071rb-freertos/CMakeLists.txt`
2. Add: `target_compile_definitions(tsumikoro_bus PRIVATE TSUMIKORO_HAL_DEBUG=1 TSUMIKORO_BUS_DEBUG=1)`
3. Rebuild and flash:
```bash
cd firmware
make clean-nucleo-g071rb-freertos
make nucleo-g071rb-freertos flash-nucleo-g071rb-freertos
```

### Debug hard fault:
```bash
# Terminal 1: st-util
# Terminal 2:
arm-none-eabi-gdb nucleo-g071rb-freertos/nucleo-g071rb-freertos.elf
(gdb) target extended-remote :4242
(gdb) continue
# Wait for hard fault
(gdb) bt          # Backtrace
(gdb) info registers
(gdb) x/32x $sp   # Examine stack
```

## Continuous Integration

The project uses GitHub Actions for CI/CD. All builds and tests run automatically on push to main and pull requests.

### CI Jobs
- **test-protocol**: Run bus protocol unit tests
- **build-stm32-ministepper**: Build ministepper (STM32G071G8U6)
- **build-stm32-servo**: Build servo (STM32G030F6P6)
- **build-nucleo-g071rb**: Build NUCLEO bare-metal (STM32G071RBT6)
- **build-nucleo-g071rb-freertos**: Build NUCLEO FreeRTOS (STM32G071RBT6)
- **build-esp32-bridge**: Build ESP32 bridge (ESP32-S3)

### CI Artifacts
Build artifacts are retained for 30 days and include:
- `.elf` files (with debug symbols)
- `.bin` files (flashable binaries)
- `.hex` files (Intel HEX format)

### Keeping CI in Sync
**IMPORTANT**: When adding or removing firmware targets, update both:
1. `.github/workflows/build.yml` - Add CI build job
2. `firmware/Makefile` - Add build and flash targets

Both files have comments at the top reminding you to keep them synchronized.

### Running CI Locally
```bash
cd firmware
make test                          # Run protocol tests
make ministepper-g071              # Build ministepper
make servo-g030                    # Build servo
make nucleo-g071rb                 # Build NUCLEO bare-metal
make nucleo-g071rb-freertos        # Build NUCLEO FreeRTOS
```

## Troubleshooting

### Firmware doesn't start after flash
- Make sure you used `--reset` flag
- Try power cycling the board
- Check with GDB if stuck in Error_Handler or HardFault_Handler

### No RX activity
- Enable TSUMIKORO_HAL_DEBUG to see if data is being received
- Check DMA and UART IRQ counts in Stats output
- Verify UART1 is properly initialized (1 Mbaud, 8N1)

### Stack overflow
- Check Stats task output for stack high water marks
- Increase stack size in xTaskCreate() or bus thread configs
- Look for large stack buffers in functions

### Heap exhaustion
- Check "Heap free" in Stats output
- Reduce configTOTAL_HEAP_SIZE or task stack sizes if needed
- Verify no memory leaks in dynamic allocations