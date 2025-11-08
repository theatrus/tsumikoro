# NUCLEO-G071RB FreeRTOS Example

This is a FreeRTOS-enabled version of the NUCLEO-G071RB development board firmware, demonstrating the **RTOS mode** of the Tsumikoro bus protocol.

## Overview

This firmware demonstrates:
- **FreeRTOS integration** with the Tsumikoro bus protocol
- **RTOS mode** bus protocol with dedicated threads for RX, TX, and command handling
- **Multi-tasking** with separate tasks for LED control, button monitoring, and statistics
- **Thread-safe** LED state management using mutexes
- **Asynchronous bus processing** without requiring `tsumikoro_bus_process()` polling

## Differences from Bare-Metal Version

| Feature | Bare-Metal (`nucleo-g071rb`) | FreeRTOS (`nucleo-g071rb-freertos`) |
|---------|------------------------------|-------------------------------------|
| **Bus Mode** | Bare-metal polling | RTOS mode with threads |
| **Bus Processing** | Manual `tsumikoro_bus_process()` calls | Automatic (handled by bus threads) |
| **Task Structure** | Single `while(1)` loop | Multiple FreeRTOS tasks |
| **LED Control** | Direct in main loop | Dedicated task with mutex |
| **Button Monitoring** | Polled in main loop | Dedicated task |
| **Synchronization** | Simple flags | FreeRTOS mutexes and semaphores |
| **Memory Overhead** | ~0 KB | ~12 KB heap + task stacks |
| **Responsiveness** | Depends on loop timing | Real-time with priorities |

## Features

### Hardware
- **LED (LD4)**: PA5 - Auto-blinks by default, controllable via bus
- **Button (B1)**: PC13 - Monitored for press events
- **Debug UART**: USART2 (115200 baud) via ST-Link VCP
- **Bus Interface**: USART1 (1 Mbaud) - PC4(TX), PC5(RX), PA1(DE)

### FreeRTOS Tasks

#### 1. LED Task (Priority: Low)
- Runs every 1000 ms
- Handles auto-blink mode (toggles LED when enabled)
- Uses mutex for thread-safe LED state access

#### 2. Button Task (Priority: Low)
- Polls button every 50 ms
- Detects and reports button presses

#### 3. Statistics Task (Priority: Low)
- Prints status every 10 seconds
- Shows interrupt counts, callback counts, and heap usage

#### 4. Bus Protocol Tasks (Created by bus library)
- **RX Thread** (High Priority): Processes incoming UART data
- **TX Thread** (High Priority): Handles transmission requests
- **Handler Thread** (Normal Priority): Manages bus state machine and callbacks

### Bus Commands

| Command | Value | Description | Data |
|---------|-------|-------------|------|
| PING | 0x0000 | Simple ping | None |
| GET_VERSION | 0x0001 | Get firmware version | Returns: [major, minor, patch, reserved] |
| GET_STATUS | 0x0002 | Get device status | Returns: [status, LED state, button state] |
| LED_SET | 0xF001 | Set LED state | Send: [state] (0=OFF, 1=ON, 2=AUTO-BLINK) |
| LED_GET | 0xF002 | Get LED state | Returns: [LED state, auto-blink enabled] |
| BUTTON_GET | 0xF003 | Get button state | Returns: [button state] (0=released, 1=pressed) |

## Building

### Using Make:
```bash
cd firmware
make nucleo-g071rb-freertos
```

### Using CMake directly:
```bash
mkdir -p build/nucleo-g071rb-freertos
cd build/nucleo-g071rb-freertos
cmake ../.. -DPROJECT=nucleo-g071rb-freertos -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Flashing

### Using st-flash (Linux):
```bash
cd build/nucleo-g071rb-freertos
st-flash --reset write nucleo-g071rb-freertos.bin 0x8000000
```

### Using Make target:
```bash
cd build/nucleo-g071rb-freertos
make flash
```

## Memory Usage

```
STM32G071RBT6: 128 KB Flash, 36 KB RAM

Approximate Usage:
- Flash: ~40 KB (code + FreeRTOS)
- RAM:   ~18 KB (heap + stacks + BSS)
- Heap:   12 KB (FreeRTOS heap_4)
```

## FreeRTOS Configuration

Key settings in `FreeRTOSConfig.h`:

- **CPU Clock**: 64 MHz
- **Tick Rate**: 1 kHz (1 ms tick)
- **Max Priorities**: 5
- **Heap Size**: 12 KB
- **Stack Overflow Detection**: Enabled (method 2)
- **Cortex-M0+ Port**: Uses primask (no basepri register)

## Testing with ESP32 Bridge

1. Flash the NUCLEO board with this firmware
2. Connect to ESP32 bridge via RS-485
3. Use ESPHome controls to interact with the board

Example bus topology:
```
ESP32-S3 (Controller, ID=0x00) <--RS-485--> NUCLEO-G071RB (Peripheral, ID=0x10)
   |
   WiFi
   |
Home Assistant
```

## Debug Output

Connect to ST-Link VCP at 115200 baud to see:
- Startup banner with FreeRTOS version
- Bus command logs
- Button press events
- Periodic statistics (every 10 seconds)

Example output:
```
========================================
NUCLEO-G071RB Tsumikoro DevBoard
STM32G071RBT6 @ 64 MHz
128KB Flash, 36KB RAM
FreeRTOS v11.1.0
Bus ID: 0x10, Baud: 1 Mbaud
USART1: PC4(TX/D1), PC5(RX/D0), PA1(DE)
RTOS Mode: Enabled
========================================

[BUS] Initialized in RTOS mode
[MAIN] Starting FreeRTOS scheduler...

Uptime: 10 s, LED: ON
  UART IRQ: 45, DMA RX: 12, DMA TX: 8
  Callbacks: 4, Heap free: 9856 bytes

Button pressed!
[BUS] RX: dev=0x10 cmd=0xF002 len=0
[BUS] LED Get: state=1 auto=1
```

## Why Use RTOS Mode?

### Advantages:
1. **True multi-tasking**: Bus processing doesn't block application tasks
2. **Better responsiveness**: High-priority RX thread handles incoming data immediately
3. **Cleaner code**: Each feature in its own task
4. **Thread-safe**: Built-in synchronization primitives
5. **Scalable**: Easy to add new tasks without refactoring

### Disadvantages:
1. **Memory overhead**: ~12-18 KB for FreeRTOS
2. **Complexity**: More concepts to understand (tasks, mutexes, queues)
3. **Debugging**: Multi-threaded debugging is harder
4. **Timing**: Scheduler introduces small overhead

### When to Use:
- Complex applications with multiple concurrent features
- When bus latency must be minimal
- Applications that need real-time guarantees
- Projects with sufficient RAM (>16 KB recommended)

### When NOT to Use:
- Simple applications with one main feature
- Memory-constrained devices (<16 KB RAM)
- Single-threaded workflows
- When bare-metal performance is critical

## License

Apache License 2.0 - See LICENSE file in repository root

## See Also

- **Bare-metal version**: `nucleo-g071rb/` - Simpler, less memory overhead
- **Bus protocol documentation**: `shared/tsumikoro_bus/README.md`
- **FreeRTOS documentation**: https://www.freertos.org/
