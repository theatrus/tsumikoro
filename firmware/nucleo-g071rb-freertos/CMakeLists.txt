# nucleo-g071rb-freertos project (with FreeRTOS support)

# Project sources
set(PROJECT_SOURCES
    src/main.c
    src/stm32g0xx_it.c
    src/system_stm32g0xx.c
)

# Startup file (STM32G071RBT6: 128KB Flash, 36KB RAM, LQFP64 package)
set(STARTUP_FILE
    startup/startup_stm32g071rbtx.s
)

# Linker script
set(LINKER_SCRIPT
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32G071RBT6_FLASH.ld
)

# Include FreeRTOS CMake configuration
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/FreeRTOS_STM32G0.cmake)

# FreeRTOS needs to know where the config is
target_include_directories(freertos_stm32g0 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES} ${STARTUP_FILE})

# Provide HAL config to the HAL library
target_include_directories(stm32g0_hal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Enable FreeRTOS in tsumikoro_bus (use RTOS mode)
set(TSUMIKORO_BUS_USE_RTOS ON CACHE BOOL "Enable FreeRTOS in bus protocol" FORCE)

# Add tsumikoro_bus library in RTOS mode
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_bus ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_bus)

# Enable bus protocol debug output
target_compile_definitions(tsumikoro_bus PRIVATE
    TSUMIKORO_BUS_DEBUG
    TSUMIKORO_HAL_MOCK_DEBUG
)

# tsumikoro_bus needs HAL config and FreeRTOS headers
target_include_directories(tsumikoro_bus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Link FreeRTOS to tsumikoro_bus so it can find the headers
target_link_libraries(tsumikoro_bus PUBLIC freertos_stm32g0)

# Link libraries (add FreeRTOS)
target_link_libraries(${PROJECT_NAME}.elf
    stm32g0_hal
    freertos_stm32g0
    tsumikoro_bus
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

# Flash target for ST-Link on Linux
add_custom_target(flash
    COMMAND st-flash --reset write ${PROJECT_NAME}.bin 0x8000000
    DEPENDS ${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Flashing ${PROJECT_NAME}.bin to target via ST-Link"
)
