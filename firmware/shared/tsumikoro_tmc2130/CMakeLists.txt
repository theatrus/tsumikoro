# TMC2130 Stepper Motor Driver Library

add_library(tsumikoro_tmc2130 STATIC
    src/tsumikoro_tmc2130.c
)

target_include_directories(tsumikoro_tmc2130
    PUBLIC
        include
)

# Platform-independent library - no HAL dependencies
# SPI and GPIO callbacks must be provided by the application
