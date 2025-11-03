# STM32G071CBU6 Project Template

This template is configured for the **STM32G071CBU6** microcontroller:

## Specifications

- **Part Number**: STM32G071CBU6
- **Core**: ARM Cortex-M0+ @ 64MHz
- **Flash**: 128KB
- **RAM**: 36KB
- **Package**: UFQFPN48 (7x7mm, 0.5mm pitch)

## Using This Template

To create a new project based on this template:

1. Copy this entire directory to your project location:
   ```bash
   cp -r common/templates/stm32g071cbu6 your-project-name
   ```

2. Update the CMakeLists.txt with your project name and source files

3. Add your source code to `src/` and headers to `inc/`

4. Copy the HAL configuration from an existing project:
   ```bash
   cp tsumikoro-ministepper/inc/stm32g0xx_hal_conf.h your-project-name/inc/
   ```

5. Build your project:
   ```bash
   mkdir -p build/your-project-name
   cd build/your-project-name
   cmake ../.. -DPROJECT=your-project-name -DMCU=STM32G071 -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   ```

## Files Included

- `startup/startup_stm32g071cbu6.s` - GCC startup code
- `linker/STM32G071CBU6_FLASH.ld` - Linker script with correct memory map
- `CMakeLists.txt` - Build configuration template
