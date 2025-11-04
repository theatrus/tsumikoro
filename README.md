# Tsumikoro

Open-source motor controller projects based on STM32G0 microcontrollers with ESP32 network bridge.

## Projects

| Project | Description | MCU | Flash/RAM | Framework | Status |
|---------|-------------|-----|-----------|-----------|--------|
| **tsumikoro-servo** | 6-channel servo + H-bridge motor controller | STM32G030F6P6TR | 32KB/8KB | STM32 LL | ✅ Functional (PR #9) |
| **tsumikoro-ministepper** | Mini stepper motor controller | STM32G071G8U6 | 64KB/36KB | STM32 HAL | 🚧 In Development |
| **tsumikoro-bridge** | WiFi/Network bridge (RS-485 bus) | ESP32/ESP32-S3 | 4MB+ | ESPHome | ✅ Functional (Merged) |

### Recent Updates

- **2025-11**: Multi-drop RS-485 bus protocol implemented with ESP32 HAL
- **2025-11**: Servo controller: 6 PWM channels + H-bridge motor driver (89% Flash usage)
- **2025-11**: ESP32 bridge with Home Assistant integration
- **2025-11**: Unified build system with top-level Makefile

## Repository Structure

```
tsumikoro/
├── firmware/                 # All firmware projects (C/C++, Apache 2.0)
│   ├── tsumikoro-ministepper/  # STM32G071 mini stepper controller
│   ├── tsumikoro-servo/        # STM32G030 servo controller
│   ├── tsumikoro-bridge/       # ESP32/ESP32-S3 network bridge (ESPHome)
│   │   ├── esphome/           # ESPHome configuration
│   │   └── components/        # Custom ESPHome components
│   ├── common/                # Shared HAL and utilities (STM32)
│   └── shared/                # Shared protocol definitions (C/C++)
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

# Clean all builds
make clean

# View all available targets
make help
```

### Individual Project Builds

**Servo Controller (STM32G030):**
```bash
make build-servo
# Output: firmware/build/servo-g030/tsumikoro-servo.elf
```

**Ministepper (STM32G071):**
```bash
make build-ministepper
# Output: firmware/build/ministepper-g071/tsumikoro-ministepper.elf
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

## Hardware

### Motor Controllers (STM32)

**STM32G071G8U6** (Mini Stepper)
- 64KB Flash, 36KB RAM
- ARM Cortex-M0+ @ 64MHz
- UFQFPN28 package

**STM32G030F6P6TR** (Servo)
- 32KB Flash, 8KB RAM
- ARM Cortex-M0+ @ 64MHz
- TSSOP20 package

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
