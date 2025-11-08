# FreeRTOS for STM32G0 (Cortex-M0+)

set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32CubeG0/Middlewares/Third_Party/FreeRTOS/Source)

# FreeRTOS core sources
set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

# Create FreeRTOS library
add_library(freertos_stm32g0 STATIC ${FREERTOS_SOURCES})

# Include directories
target_include_directories(freertos_stm32g0 PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM0
)

# Compiler flags for FreeRTOS
target_compile_options(freertos_stm32g0 PRIVATE
    -Wno-unused-parameter
)
