# tsumikoro-servo project

# Project sources
set(PROJECT_SOURCES
    src/main.c
    src/servo_pwm.c
    src/motor_hbridge.c
    src/limit_switch.c
    src/system_stm32g0xx.c
)

# Startup file (STM32G030F6P6: 32KB Flash, 8KB RAM, TSSOP20 package)
set(STARTUP_FILE
    startup/startup_stm32g030f6p6.s
)

# Linker script
set(LINKER_SCRIPT
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32G030F6P6_FLASH.ld
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES} ${STARTUP_FILE})

# Provide HAL config to the HAL library
target_include_directories(stm32g0_hal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Explicitly disable FreeRTOS in tsumikoro_bus (use bare-metal polling mode)
set(TSUMIKORO_BUS_USE_RTOS OFF CACHE BOOL "Disable FreeRTOS in bus protocol" FORCE)

# Add tsumikoro_bus library in bare-metal mode
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_bus ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_bus)

# Add tsumikoro_flash_config library for persistent configuration
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_flash_config ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_flash_config)

# tsumikoro_bus needs HAL config for its STM32 HAL implementation
target_include_directories(tsumikoro_bus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# tsumikoro_flash_config needs CMSIS device headers and STM32 target definition
target_include_directories(tsumikoro_flash_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/STM32CubeG0/Drivers/CMSIS/Device/ST/STM32G0xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/STM32CubeG0/Drivers/CMSIS/Include
)
target_compile_definitions(tsumikoro_flash_config PRIVATE STM32G030xx)

# Link libraries
target_link_libraries(${PROJECT_NAME}.elf
    stm32g0_hal
    tsumikoro_bus
    tsumikoro_flash_config
)

# Include directories
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    inc
)

# Linker script
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map
)

# Post-build: Generate hex and bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Generating HEX and BIN files, showing size"
)
