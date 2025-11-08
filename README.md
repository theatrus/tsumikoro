# Tsumikoro

Open-source motor controller projects based on STM32G0 microcontrollers with ESP32 network bridge.

## Projects

| Project | Description | MCU | Flash/RAM | Framework | Status |
|---------|-------------|-----|-----------|-----------|--------|
| **tsumikoro-servo** | 6-channel servo + H-bridge motor controller | STM32G030F6P6TR | 32KB/8KB | STM32 LL | ✅ Functional (PR #9) |
| **tsumikoro-ministepper** | Mini stepper motor controller | STM32G071G8U6 | 64KB/36KB | STM32 HAL + FreeRTOS | 🚧 In Development |
| **tsumikoro-bridge** | WiFi/Network bridge (RS-485 bus) | ESP32/ESP32-S3 | 4MB+ | ESPHome | ✅ Functional (Merged) |
| **nucleo-g071rb** | Development firmware (bare-metal) | STM32G071RBT6 | 128KB/36KB | STM32 HAL | ✅ Development Tool (PR #11) |
| **nucleo-g071rb-freertos** | Development firmware (FreeRTOS) | STM32G071RBT6 | 128KB/36KB | STM32 HAL + FreeRTOS | ✅ Development Tool (PR #11) |

### Recent Updates

- **2025-11**: NUCLEO-G071RB development firmware with FreeRTOS integration (PR #11)
  - Bare-metal and FreeRTOS examples for bus protocol development
  - Stack monitoring and optimization (18KB heap, optimized task stacks)
  - Debug UART with statistics output and stack high-water marks
  - CI/CD integration for all firmware targets
- **2025-11**: Protocol improvements and bug fixes
  - Fixed escape sequence handling in packet END marker detection
  - Added comprehensive unit tests for bus handler (34 tests)
  - Integration tests for controller/peripheral communication
- **2025-11**: Multi-drop RS-485 bus protocol implemented with STM32 HAL
- **2025-11**: Servo controller: 6 PWM channels + H-bridge motor driver (89% Flash usage)
- **2025-11**: ESP32 bridge with Home Assistant integration
- **2025-11**: Unified build system with top-level Makefile and flash targets

## Repository Structure

```
tsumikoro/
├── firmware/                 # All firmware projects (C/C++, Apache 2.0)
│   ├── tsumikoro-ministepper/  # STM32G071 mini stepper controller (FreeRTOS)
│   ├── tsumikoro-servo/        # STM32G030 servo controller (bare-metal)
│   ├── tsumikoro-bridge/       # ESP32/ESP32-S3 network bridge (ESPHome)
│   │   ├── esphome/           # ESPHome configuration
│   │   └── components/        # Custom ESPHome components
│   ├── nucleo-g071rb/         # NUCLEO-G071RB development firmware (bare-metal)
│   ├── nucleo-g071rb-freertos/# NUCLEO-G071RB development firmware (FreeRTOS)
│   ├── common/                # Shared HAL and utilities (STM32, FreeRTOS)
│   └── shared/                # Shared protocol definitions and tests
│       └── tsumikoro_bus/    # Bus protocol library with unit tests
├── hardware/                 # PCB designs (coming soon, CERN-OHL-P v2)
├── mechanical/               # 3D models (coming soon, CERN-OHL-P v2)
└── LICENSE                  # Dual licensing information
```

## Quick Start

### Prerequisites

**For STM32 builds:**
- ARM GCC toolchain: `arm-none-eabi-gcc`
- CMake 3.20 or newer
- Make or Ninja build tool

**For ESP32 bridge:**
- Python 3.10+
- uv (Python package manager)

See [firmware/README.md](firmware/README.md) for detailed installation instructions.

### Build All Projects

```bash
# First-time setup
make setup

# Build everything
make build

# Or build individual projects
make build-servo       # STM32G030 servo controller
make build-ministepper # STM32G071 stepper controller
make build-bridge      # ESP32 network bridge
make build-nucleo      # NUCLEO-G071RB development boards

# Run protocol tests
make test

# Clean all builds
make clean

# View all available targets
make help
```

### Individual Project Builds

**Servo Controller (STM32G030):**
```bash
cd firmware
make servo-g030
make flash-servo-g030  # Flash using st-flash
# Output: build/servo-g030/tsumikoro-servo.bin
```

**Ministepper (STM32G071):**
```bash
cd firmware
make ministepper-g071
make flash-ministepper-g071
# Output: build/ministepper-g071/tsumikoro-ministepper.bin
```

**NUCLEO-G071RB Development Boards:**
```bash
cd firmware

# Bare-metal version
make nucleo-g071rb
make flash-nucleo-g071rb

# FreeRTOS version (with stack monitoring)
make nucleo-g071rb-freertos
make flash-nucleo-g071rb-freertos

# Reset target after flashing
make reset
```

**ESP32 Bridge:**
```bash
# First time setup
cd firmware/tsumikoro-bridge
make setup
cp esphome/secrets.yaml.example esphome/secrets.yaml
# Edit secrets.yaml with your WiFi credentials

# Build and upload
make build
make upload
```

## Key Features

### Tsumikoro Bus Protocol

Multi-drop RS-485 serial bus protocol for reliable motor controller communication:

- **Frame Format**: `[START][ID][CMD_HI][CMD_LO][LEN][DATA][CRC8][END]`
- **Packet Size**: 7-71 bytes (0-64 bytes data payload)
- **Addressing**: Controller (0x00) + up to 239 peripherals (0x01-0xEF)
- **Error Detection**: CRC8 checksum with frame markers
- **Command Ranges**:
  - `0x0000-0x0FFF`: Generic commands (ping, version, status)
  - `0x2000-0x2FFF`: Stepper motor commands
  - `0x3000-0x3FFF`: Servo motor commands
  - `0x4000-0x4FFF`: DC motor/H-bridge commands

### Servo Controller Features

**Hardware Capabilities:**
- 6 independent servo channels (50Hz PWM, 1-2ms pulse width)
- H-bridge motor driver (20kHz PWM speed control)
- 4 direction modes: FORWARD, REVERSE, BRAKE, COAST
- 0-180° servo position range (0.1° resolution)

**Control Features:**
- Smooth position transitions with configurable speed
- Per-channel calibration (500-2500µs pulse widths)
- Multi-servo synchronized movement
- Emergency stop function
- Hardware ID pins for device enumeration

**Resource Efficient:**
- Flash: 29.3KB / 32KB (89.42%)
- RAM: 2.2KB / 8KB (26.86%)
- Zero CPU overhead for PWM (hardware timers)

### ESP32 Bridge Features

- WiFi 802.11 b/g/n connectivity
- Home Assistant integration via ESPHome
- RS-485 bus communication with motor controllers
- Custom ESPHome components for Tsumikoro protocol
- OTA firmware updates

### FreeRTOS Integration

**Real-Time Operating System Support:**
- Preemptive multitasking with configurable priorities
- Dedicated RTOS threads for RX, TX, and bus handler
- Inter-task communication via queues and semaphores
- Stack overflow detection and monitoring
- Optimized heap and stack allocations (18KB heap, task-specific stacks)

**Development Tools:**
- Stack high-water mark monitoring via debug UART
- Runtime statistics (heap usage, IRQ counts, task states)
- Comprehensive unit tests for protocol and bus handler
- CI/CD integration for automated builds and testing

### Continuous Integration

**Automated Builds:**
- All firmware targets built automatically on push/PR
- Protocol unit tests run on every commit
- Artifacts retained for 30 days
- Multi-platform support (STM32 + ESP32)

**Build Matrix:**
- STM32 builds: servo-g030, ministepper-g071, nucleo-g071rb, nucleo-g071rb-freertos
- ESP32 builds: tsumikoro-bridge (ESPHome)
- Protocol tests: 151 assertions across CRC8, protocol, integration, and bus handler tests

## Hardware

### Motor Controllers (STM32)

**STM32G071G8U6** (Mini Stepper)
- 64KB Flash, 36KB RAM
- ARM Cortex-M0+ @ 64MHz
- UFQFPN28 package
- FreeRTOS support with optimized task stacks

**STM32G030F6P6TR** (Servo)
- 32KB Flash, 8KB RAM
- ARM Cortex-M0+ @ 64MHz
- TSSOP20 package
- Bare-metal for maximum efficiency

### Development Boards

**STM32 NUCLEO-G071RB**
- 128KB Flash, 36KB RAM
- ARM Cortex-M0+ @ 64MHz
- Arduino-compatible headers
- On-board ST-LINK debugger
- Available in both bare-metal and FreeRTOS firmware variants
- Ideal for bus protocol development and testing

### Network Bridge (ESP32)

**ESP32 / ESP32-S3**
- WiFi 802.11 b/g/n
- 4MB+ Flash recommended
- UART communication with motor controllers
- Home Assistant integration via ESPHome

## License

This project uses dual licensing:

- **Firmware/Software**: [Apache 2.0](firmware/LICENSE)
- **Hardware Designs**: CERN-OHL-P v2 (permissive open hardware)

See [LICENSE](LICENSE) for details.

## Contributing

Contributions are welcome! Please ensure:
- Firmware contributions follow the Apache 2.0 license
- Hardware contributions follow CERN-OHL-P v2
- Code follows the existing style
- All changes are tested

## Resources

### Documentation
- [STM32 Firmware Documentation](firmware/README.md)
- [ESP32 Bridge Documentation](firmware/tsumikoro-bridge/README.md)

### Hardware
- [STM32G0 Series](https://www.st.com/en/microcontrollers-microprocessors/stm32g0-series.html)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/)
- [ESPHome](https://esphome.io/)

### Licenses
- [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0)
- [CERN-OHL-P v2](https://ohwr.org/cern_ohl_p_v2.txt)

### Tools
- [uv Python Package Manager](https://github.com/astral-sh/uv)

---

Copyright (c) 2025-2025 Yann Ramin
