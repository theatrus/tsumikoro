# tsumikoro-ministepper project

# Project sources
set(PROJECT_SOURCES
    src/main.c
    src/system_stm32g0xx.c
)

# Startup file (STM32G071G8U6: 64KB Flash, 36KB RAM, UFQFPN28 package)
set(STARTUP_FILE
    startup/startup_stm32g071g8u6.s
)

# Linker script
set(LINKER_SCRIPT
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32G071G8U6_FLASH.ld
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES} ${STARTUP_FILE})

# Provide HAL config to the HAL library
target_include_directories(stm32g0_hal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Enable FreeRTOS support in tsumikoro_bus
set(TSUMIKORO_BUS_USE_RTOS ON CACHE BOOL "Enable FreeRTOS in bus protocol" FORCE)

# FreeRTOS sources (define before adding tsumikoro_bus)
set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32CubeG0/Middlewares/Third_Party/FreeRTOS/Source)
set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0/port.c
)

# Create FreeRTOS library
add_library(freertos STATIC ${FREERTOS_SOURCES})
target_include_directories(freertos PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_compile_definitions(freertos PUBLIC STM32G0)

# Add tsumikoro_bus library AFTER FreeRTOS is defined
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_bus ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_bus)

# tsumikoro_bus needs HAL config and FreeRTOS includes
target_include_directories(tsumikoro_bus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(tsumikoro_bus PRIVATE freertos)

# Link libraries
target_link_libraries(${PROJECT_NAME}.elf
    stm32g0_hal
    tsumikoro_bus
    freertos
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
