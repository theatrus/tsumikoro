# Tsumikoro Firmware

STM32G0-based firmware for Tsumikoro motor controller projects.

## Hardware

### Current Projects

| Project | MCU | Flash | RAM | Package | Status | Description |
|---------|-----|-------|-----|---------|--------|-------------|
| **tsumikoro-servo** | STM32G030F6P6TR | 32KB | 8KB | TSSOP20 | ✅ Functional | 6-channel servo + H-bridge motor controller |
| **tsumikoro-ministepper** | STM32G071G8U6 | 64KB | 36KB | UFQFPN28 | 🚧 Development | Mini stepper motor controller |

### Supported MCUs

- **STM32G071G8U6**: Cortex-M0+, 64KB Flash, 36KB RAM, UFQFPN28
- **STM32G030F6P6**: Cortex-M0+, 32KB Flash, 8KB RAM, TSSOP20
- **STM32G071CBU6**: Cortex-M0+, 128KB Flash, 36KB RAM, UFQFPN48 (template available in `common/templates/`)

## Project Structure

```
firmware/
├── CMakeLists.txt              # Top-level build configuration (STM32 only)
├── cmake/                      # Build system configuration (STM32)
│   ├── arm-none-eabi.cmake    # ARM GCC toolchain
│   ├── stm32g071.cmake        # STM32G071 MCU configuration
│   └── stm32g030.cmake        # STM32G030 MCU configuration
├── common/                     # Shared STM32 libraries and code
│   ├── CMakeLists.txt         # Common library build config
│   ├── STM32CubeG0/           # ST HAL/LL drivers (submodule)
│   └── templates/             # MCU-specific project templates
│       └── stm32g071cbu6/     # STM32G071CBU6 template
├── shared/                     # Shared protocol definitions (C/C++)
│   └── tsumikoro_protocol.h   # Communication protocol (ESP32 + STM32)
├── tsumikoro-ministepper/     # Mini stepper (STM32G071G8U6)
│   ├── src/                   # Source files
│   ├── inc/                   # Header files
│   ├── startup/               # Startup assembly
│   ├── linker/                # Linker scripts
│   └── CMakeLists.txt         # Project build config
├── tsumikoro-servo/           # Servo controller (STM32G030F6P6)
│   ├── src/                   # Source files
│   ├── inc/                   # Header files
│   ├── startup/               # Startup assembly
│   ├── linker/                # Linker scripts
│   └── CMakeLists.txt         # Project build config
└── tsumikoro-bridge/          # ESP32/ESP32-S3 network bridge (ESPHome)
    ├── esphome/               # ESPHome configuration
    ├── components/            # Custom ESPHome components
    ├── Makefile               # Build system (uses uv)
    └── pyproject.toml         # Python dependencies
```

## Project Details

### Tsumikoro-Servo (STM32G030F6P6)

**Status**: ✅ Functional (PR #9)

6-channel PWM servo controller with H-bridge motor driver optimized for the STM32G030F6P6 TSSOP20 package.

**Features:**
- 6 independent servo channels (50Hz PWM, 1-2ms pulse width)
- H-bridge motor driver (20kHz PWM speed control)
- 4 direction modes: FORWARD, REVERSE, BRAKE, COAST
- Hardware ID support (2-bit = 4 unique addresses)
- Multi-drop RS-485 bus communication
- Resource efficient: 89.4% Flash (29.3KB), 26.9% RAM (2.2KB)

**Pin Allocation:**

```
STM32G030F6P6 TSSOP20
============================

         ┌─────────────┐
  PB7 ──│1   •    20│── PA14 (SWCLK)
  PB8 ──│2        19│── PA13 (SWDIO)
  PB9 ──│3        18│── PA12 (Motor IN1)
 NRST ──│4        17│── PA11 (Motor PWM)
 VDDA ──│5        16│── PA8/PB0/PB1/PB2*
  PA0 ──│6        15│── PA7 (Servo 5)
  PA1 ──│7        14│── PA6 (Servo 4)
  PA2 ──│8        13│── PA5 (LED)
  GND ──│9        12│── PA4 (Servo 3)
  VDD ──│10       11│── PA3 (Servo 2)
         └─────────────┘

*Bonded pins - only one can be used
```

**Resource Allocation:**

| Resource | Usage |
|----------|-------|
| Servo Channels | PA0, PA2, PA3, PA4, PA6, PA7 |
| Motor Control | PA11 (PWM), PA12 (IN1), PB9 (IN2) |
| Hardware ID | PB7, PB8 (2-bit input) |
| Bus Communication | PA1 (DE), PA9 (TX), PA10 (RX) |
| Debug | PA13 (SWDIO), PA14 (SWCLK) |
| Status | PA5 (LED) |
| Available | Pin 16 (PA8/PB0/PB1/PB2 bonded) |

**Timers:**
- TIM1: Motor PWM (20kHz)
- TIM3: Servos 0, 1, 2
- TIM14: Servo 3
- TIM16: Servo 4
- TIM17: Servo 5

All timers allocated - no expansion possible.

See [tsumikoro-servo/README.md](tsumikoro-servo/README.md) for complete documentation.

### Tsumikoro-Ministepper (STM32G071G8U6)

**Status**: 🚧 In Development

Mini stepper motor controller (details coming soon).

## Prerequisites

### Required Tools

1. **ARM GCC Toolchain**: `arm-none-eabi-gcc`
   - Fedora: `sudo dnf install arm-none-eabi-gcc-cs arm-none-eabi-newlib`
   - Ubuntu/Debian: `sudo apt install gcc-arm-none-eabi`
   - macOS: `brew install --cask gcc-arm-embedded`

2. **CMake**: Version 3.20 or newer
   - Fedora: `sudo dnf install cmake`
   - Ubuntu/Debian: `sudo apt install cmake`
   - macOS: `brew install cmake`

3. **Build tools**: `make` or `ninja`
   - Fedora: `sudo dnf install make ninja-build`
   - Ubuntu/Debian: `sudo apt install make ninja-build`

### Optional Tools

- **OpenOCD**: For debugging and flashing
  - `sudo dnf install openocd` (Fedora)
  - `sudo apt install openocd` (Ubuntu/Debian)

- **STM32CubeProgrammer**: ST's official programming tool

## Getting Started

### Clone the Repository

```bash
git clone <repository-url>
cd tsumikoro/firmware
git submodule update --init --recursive
```

### Building

The build system requires two parameters:
- `PROJECT`: The project to build (`tsumikoro-ministepper` or `tsumikoro-servo`)
- `MCU`: The target MCU (`STM32G071` or `STM32G030`)

#### Build tsumikoro-ministepper for STM32G071

```bash
mkdir -p build/ministepper-g071
cd build/ministepper-g071
cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071
make -j$(nproc)
```

#### Build tsumikoro-servo for STM32G030

```bash
mkdir -p build/servo-g030
cd build/servo-g030
cmake ../.. -DPROJECT=tsumikoro-servo -DMCU=STM32G030
make -j$(nproc)
```

#### Build Types

CMake supports different build types with different optimization levels. For production firmware, **Release** is recommended:

| Build Type | Flags | Use Case |
|------------|-------|----------|
| **Release** | `-Os -DNDEBUG` | **Production** - Optimized for size (smallest binary) |
| **Debug** | `-O0 -g3 -DDEBUG` | Development - No optimization, full debug info |
| **RelWithDebInfo** | `-Os -g -DNDEBUG` | Production debugging - Size optimized with debug symbols |
| **MinSizeRel** | `-Os -DNDEBUG` | Same as Release - Maximum size optimization |

**To specify a build type:**

```bash
# Release build (size-optimized, recommended for production)
cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Debug build (no optimization, best for debugging)
cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

**Note:** If no build type is specified, the build will use minimal flags without optimization.

#### Using Ninja (faster builds)

```bash
cmake ../.. -DPROJECT=tsumikoro-ministepper -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release -G Ninja
ninja
```

#### Cleaning Builds

**Clean build artifacts (keeps CMake cache):**
```bash
cd build/ministepper-g071
make clean
```

**Complete clean (removes everything, requires reconfiguration):**
```bash
# Remove entire build directory
rm -rf build/ministepper-g071

# Or remove all build directories
rm -rf build/*
```

**Clean script (from firmware root):**
```bash
# Clean all projects
./clean.sh

# Clean specific project
./clean.sh ministepper-g071
```

### Build Outputs

After a successful build, you'll find:
- `<project>.elf` - ELF executable with debug symbols
- `<project>.hex` - Intel HEX format (for flashing)
- `<project>.bin` - Raw binary format
- `<project>.map` - Memory map file

### Flashing

#### Using OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
  -c "program <project>.elf verify reset exit"
```

#### Using STM32CubeProgrammer

```bash
STM32_Programmer_CLI -c port=SWD -w <project>.hex -v -rst
```

#### Using st-flash (from stlink tools)

```bash
st-flash write <project>.bin 0x8000000
```

## Debugging

### Using OpenOCD + GDB

Terminal 1 (OpenOCD server):
```bash
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
```

Terminal 2 (GDB):
```bash
arm-none-eabi-gdb <project>.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) monitor reset halt
(gdb) continue
```

## Configuration

### MCU Selection

Projects can target different MCUs in the STM32G0 family:

- **STM32G071**: Cortex-M0+, up to 64KB Flash, 36KB RAM
- **STM32G030**: Cortex-M0+, up to 32KB Flash, 8KB RAM

Choose the appropriate MCU with the `-DMCU` flag during configuration.

### Adjusting Linker Scripts

Linker scripts are located in each project's `linker/` directory. Modify these to match your specific MCU variant's memory layout.

### HAL Configuration

HAL modules are configured in `inc/stm32g0xx_hal_conf.h` in each project. Enable or disable HAL modules by uncommenting/commenting the `#define HAL_<MODULE>_MODULE_ENABLED` lines.

### Adding HAL Modules

To add more HAL modules, edit `common/CMakeLists.txt` and add the source files:

```cmake
file(GLOB HAL_SOURCES
    # ... existing files ...
    ${HAL_DIR}/Src/stm32g0xx_hal_spi.c
    ${HAL_DIR}/Src/stm32g0xx_hal_i2c.c
)
```

Then enable them in `stm32g0xx_hal_conf.h`:

```c
#define HAL_SPI_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
```

## Adding New Projects

1. Create project directory: `mkdir tsumikoro-newproject/{src,inc,startup,linker}`
2. Copy startup and linker files from an existing project
3. Create `CMakeLists.txt` based on existing project templates
4. Add your source files to `src/` and headers to `inc/`
5. Build: `cmake ... -DPROJECT=tsumikoro-newproject -DMCU=STM32G071`

## Troubleshooting

### "arm-none-eabi-gcc not found"

Ensure the ARM GCC toolchain is installed and in your PATH:
```bash
which arm-none-eabi-gcc
arm-none-eabi-gcc --version
```

### Build fails with "undefined reference to..."

Check that:
1. Required HAL modules are enabled in `stm32g0xx_hal_conf.h`
2. Corresponding source files are added in `common/CMakeLists.txt`

### Memory overflow errors

Your program is too large for the target MCU. Either:
1. Optimize for size: Change `CMAKE_C_FLAGS_RELEASE` to `-Os`
2. Remove unused HAL modules
3. Use a larger MCU variant

## License

This firmware is licensed under the **Apache License 2.0**. See [LICENSE](LICENSE) for the full license text.

### Third-Party Components

This project uses ST's STM32CubeG0 HAL library, which is licensed under BSD-3-Clause.

### Copyright

Copyright (c) 2025-2025 Yann Ramin

## Resources

- [STM32G0 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0444-stm32g0x0-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32G071 Datasheet](https://www.st.com/resource/en/datasheet/stm32g071c6.pdf)
- [STM32G030 Datasheet](https://www.st.com/resource/en/datasheet/stm32g030c6.pdf)
- [STM32CubeG0 on GitHub](https://github.com/STMicroelectronics/STM32CubeG0)
