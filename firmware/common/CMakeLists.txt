# Common library with ST HAL drivers and shared utilities

# STM32CubeG0 paths
set(CUBE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/STM32CubeG0)
set(HAL_DIR ${CUBE_DIR}/Drivers/STM32G0xx_HAL_Driver)
set(CMSIS_DIR ${CUBE_DIR}/Drivers/CMSIS)

# HAL source files - add more as needed
file(GLOB HAL_SOURCES
    ${HAL_DIR}/Src/stm32g0xx_hal.c
    ${HAL_DIR}/Src/stm32g0xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32g0xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32g0xx_hal_dma.c
    ${HAL_DIR}/Src/stm32g0xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32g0xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_tim.c
    ${HAL_DIR}/Src/stm32g0xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_uart.c
    ${HAL_DIR}/Src/stm32g0xx_hal_uart_ex.c
    ${HAL_DIR}/Src/stm32g0xx_hal_spi.c
    ${HAL_DIR}/Src/stm32g0xx_hal_spi_ex.c
)

# Create HAL library
add_library(stm32g0_hal STATIC ${HAL_SOURCES})

target_include_directories(stm32g0_hal PUBLIC
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32G0xx/Include
    ${CMSIS_DIR}/Include
)

# Allow projects to add their HAL config directory
# Projects should use: target_include_directories(stm32g0_hal PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
# Then add tsumikoro_bus after setting HAL config

# Common utilities library (if needed)
# add_library(tsumikoro_common STATIC
#     utils/common_utils.c
# )
#
# target_include_directories(tsumikoro_common PUBLIC
#     utils
# )
