# Tsumikoro bus protocol library

# Feature flags
option(TSUMIKORO_BUS_USE_RTOS "Enable FreeRTOS threading for bus protocol" OFF)

# Source files
set(TSUMIKORO_BUS_SOURCES
    src/tsumikoro_crc8.c
    src/tsumikoro_protocol.c
    src/tsumikoro_bus.c
)

# RTOS support (optional)
if(TSUMIKORO_BUS_USE_RTOS)
    list(APPEND TSUMIKORO_BUS_SOURCES src/tsumikoro_rtos.c)
    message(STATUS "Tsumikoro Bus: FreeRTOS support enabled")
endif()

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

# Add preprocessor definition if RTOS is enabled
if(TSUMIKORO_BUS_USE_RTOS)
    target_compile_definitions(tsumikoro_bus PUBLIC
        TSUMIKORO_BUS_USE_RTOS=1
    )
endif()

# This library depends on STM32 HAL
# Note: Projects using this library must add their HAL config directory
# to stm32g0_hal BEFORE calling add_subdirectory for tsumikoro_bus
target_link_libraries(tsumikoro_bus PUBLIC
    stm32g0_hal
)
