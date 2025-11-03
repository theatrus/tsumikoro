# Tsumikoro

Open-source motor controller projects based on STM32G0 microcontrollers with ESP32 network bridge.

## Projects

| Project | Description | MCU | Framework | Status |
|---------|-------------|-----|-----------|--------|
| **tsumikoro-ministepper** | Mini stepper motor controller | STM32G071G8U6 | STM32 HAL | 🚧 In Development |
| **tsumikoro-servo** | Servo motor controller | STM32G030F6P6TR | STM32 HAL | 🚧 In Development |
| **tsumikoro-bridge** | WiFi/Network bridge | ESP32/ESP32-S3 | ESPHome | 🚧 In Development |

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

### STM32 Firmware Development

See [firmware/README.md](firmware/README.md) for complete build instructions.

```bash
cd firmware
git submodule update --init --recursive

# Build tsumikoro-ministepper
mkdir -p build/ministepper-g071
cd build/ministepper-g071
cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### ESP32 Bridge Development

See [firmware/tsumikoro-bridge/README.md](firmware/tsumikoro-bridge/README.md) for complete setup and build instructions.

```bash
cd firmware/tsumikoro-bridge

# Setup with uv
make setup

# Configure secrets
cp esphome/secrets.yaml.example esphome/secrets.yaml
# Edit secrets.yaml with your WiFi credentials

# Build and flash
make build
make upload
```

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
