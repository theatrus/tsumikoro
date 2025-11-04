# tsumikoro_flash_config library - Flash configuration storage
#
# Provides persistent storage of device configuration in internal flash memory.
# Uses direct register access (LL-style) for flash operations.

# Create library
add_library(tsumikoro_flash_config STATIC
    src/tsumikoro_flash_config.c
)

# Public include directories
target_include_directories(tsumikoro_flash_config PUBLIC
    include
)

# Private include directories for STM32 CMSIS headers
# These paths are relative to the shared library location
target_include_directories(tsumikoro_flash_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/STM32CubeG0/Drivers/CMSIS/Include
)
