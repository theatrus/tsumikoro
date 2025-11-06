# Trinamic TMC2130/TMC5160 Stepper Motor Driver Library

add_library(tsumikoro_tmc_stepper STATIC
    src/tsumikoro_tmc_stepper.c
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
