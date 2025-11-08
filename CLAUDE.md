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