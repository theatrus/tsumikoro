# Tsumikoro bus protocol library

# Source files
set(TSUMIKORO_BUS_SOURCES
    src/tsumikoro_crc8.c
    src/tsumikoro_protocol.c
    src/tsumikoro_bus.c
)

# HAL implementation (STM32)
set(TSUMIKORO_BUS_HAL_STM32_SOURCES
    hal/stm32/tsumikoro_hal_stm32.c
)

# Create library
add_library(tsumikoro_bus STATIC
    ${TSUMIKORO_BUS_SOURCES}
    ${TSUMIKORO_BUS_HAL_STM32_SOURCES}
)

# Public include directories
target_include_directories(tsumikoro_bus PUBLIC
    include
    hal/stm32
)

# This library depends on STM32 HAL
# Note: Projects using this library must add their HAL config directory
# to stm32g0_hal BEFORE calling add_subdirectory for tsumikoro_bus
target_link_libraries(tsumikoro_bus PUBLIC
    stm32g0_hal
)
