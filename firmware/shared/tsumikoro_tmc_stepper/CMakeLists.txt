# Trinamic TMC2130/TMC5160 Stepper Motor Driver Library

# Core platform-independent library
add_library(tsumikoro_tmc_stepper STATIC
    src/tsumikoro_tmc_stepper.c
    src/tsumikoro_tmc_motion.c
)

target_include_directories(tsumikoro_tmc_stepper
    PUBLIC
        include
)

# Platform-independent library - no HAL dependencies
# SPI and GPIO callbacks must be provided by the application
#
# Supports:
# - TMC2130: Step/Dir driver with SPI configuration
# - TMC5160: Driver with integrated motion controller

# Optional STM32 HAL platform layer
# Projects can choose to link against tsumikoro_tmc_stepper_stm32 for STM32 support
# Note: This library requires stm32g0_hal to be available
if(TARGET stm32g0_hal)
    add_library(tsumikoro_tmc_stepper_stm32 STATIC
        hal/stm32/tsumikoro_tmc_hal_stm32.c
    )

    target_link_libraries(tsumikoro_tmc_stepper_stm32
        PUBLIC
            tsumikoro_tmc_stepper
            stm32g0_hal
    )

    target_include_directories(tsumikoro_tmc_stepper_stm32
        PUBLIC
            hal/stm32
    )
endif()
