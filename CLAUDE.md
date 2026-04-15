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
|-----|-------------|----------------|--------------------------------------------|
| 1   | PB7         | DRV8876 nFAULT | GPIO input, 10k pull-up to 3V3 (open-drain) |
| 2   | PB8         | I2C1 SCL       | AF6                                        |
| 3   | PB9         | I2C1 SDA       | AF6                                        |
| 4   | NRST        | Reset          | —                                          |
| 5   | VDDA        | 3V3 analog     | —                                          |
| 6   | PA0         | Servo 0        | AF1 TIM3_CH1                               |
| 7   | PA1         | RS-485 DE      | GPIO output                                |
| 8   | PA2         | Servo 1        | AF1 TIM3_CH3                               |
| 9   | VSS         | GND            | —                                          |
| 10  | VDD         | 3V3            | —                                          |
| 11  | PA3         | Servo 2        | AF1 TIM3_CH4                               |
| 12  | PA4         | Motor PH (DIR) | GPIO output → DRV8876 PH                   |
| 13  | PA5         | Status LED     | GPIO output                                |
| 14  | PA6         | Motor EN (PWM) | AF5 TIM16_CH1 → DRV8876 EN, 20 kHz          |
| 15  | PA7         | Servo 3        | AF5 TIM17_CH1                              |
| 16  | PA8/PB0-2   | Limit switch   | GPIO input, internal pull-up                |
| 17  | PA9  (rmp)  | USART1 TX      | AF1, RS-485 @ 1 Mbaud                      |
| 18  | PA10 (rmp)  | USART1 RX      | AF1                                        |
| 19  | PA13        | SWDIO          | protected                                  |
| 20  | PA14        | SWCLK          | protected                                  |

**Important — SYSCFG remap:** On the TSSOP-20, pins 17/18 default to
PA11/PA12. To reach USART1 on PA9/PA10, firmware must set
`SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP`
*before* configuring AF1 USART1 on those pins. See `src/main.c::GPIO_Init`.

**DRV8876 strapping on this board (PH/EN mode):**
- `PMODE` → VCC (PH/EN mode)
- `nSLEEP` → VCC (always enabled; no software shutdown)
- `IMODE` → GND (current regulation disabled)
- `VREF` → GND (unused)
- `IPROPI` → no connect
- `nFAULT` → 10k pull-up to 3V3, routed to PB7

Because `nSLEEP` is hard-tied high, `MOTOR_DIR_COAST` and `MOTOR_DIR_BRAKE`
are hardware-equivalent — EN=0 gives a synchronous low-side brake. This
is documented in `inc/motor_hbridge.h`.

**Hardware ID / addressing:** There are no HW-ID strap pins on this rev.
Per-unit addressing is expected to live in the last 2 KB flash config
sector (0x08007800–0x08007FFF), provisioned at manufacturing time.

## Ministepper PCB Pin Map (STM32G071G8U6 UFQFPN-28, rev 0.1)

Custom PCB (`hardware/ministepper/`). Dual TMC2130 stepper driver node.
Uses **USART2 on PA2/PA3** — dedicated pins, no SYSCFG remap needed
(unlike the servo's G030 where PA11/PA12 shared pads with PA9/PA10).

| Pin | MCU      | Function           | AF / notes |
|-----|----------|--------------------|------------|
| 1   | VDD      | 3V3                | — |
| 2   | VDDA     | 3V3 analog         | ferrite + 100 nF |
| 3   | NRST     | Reset              | — |
| 4   | PA0      | STEP2              | AF2 TIM2_CH1 — driver 2 step pulse |
| 5   | PA1      | RS-485 DE          | GPIO output |
| 6   | PA2      | USART2 TX          | AF1 — RS-485 |
| 7   | PA3      | USART2 RX          | AF1 |
| 8   | PA4      | TMC1 CS            | GPIO output (manual SPI CS) |
| 9   | PA5      | SPI1 SCK           | AF0 — shared by both TMCs |
| 10  | PA6      | SPI1 MISO          | AF0 |
| 11  | PA7      | SPI1 MOSI          | AF0 |
| 12  | PA8      | STEP1              | AF2 TIM1_CH1 — driver 1 step pulse |
| 13  | PA11     | DRV_ENN (shared)   | GPIO out, active-low enable for both TMCs |
| 14  | PA12     | TMC2 CS            | GPIO output |
| 15  | PA13     | SWDIO              | protected |
| 16  | PA14     | SWCLK              | protected |
| 17  | PA15     | DIR1               | GPIO output |
| 18  | PB0      | Limit 1            | GPIO input, internal pull-up (switch to GND) |
| 19  | PB1      | DIAG1              | GPIO input + 47k pull-up (open-drain from TMC) |
| 20  | PB2      | DIR2               | GPIO output |
| 21  | PB3      | Limit 2            | GPIO input, internal pull-up |
| 22  | PB4      | DIAG2              | GPIO input + 47k pull-up |
| 23  | PB5      | Status LED         | GPIO output |
| 24  | PB6      | I2C1 SCL           | AF6 |
| 25  | PB7      | I2C1 SDA           | AF6 |
| 26  | PB8      | Spare              | — |
| 27  | VSS      | GND                | — |
| 28  | VDD_2    | 3V3                | second digital supply pin |

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