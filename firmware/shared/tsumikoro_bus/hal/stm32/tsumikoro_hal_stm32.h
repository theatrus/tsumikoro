/**
 * @file tsumikoro_hal_stm32.h
 * @brief STM32 HAL implementation for Tsumikoro bus
 *
 * Implements the Tsumikoro HAL interface using STM32 HAL UART with DMA.
 * Designed for STM32G0 series but should work with other STM32 families.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_HAL_STM32_H
#define TSUMIKORO_HAL_STM32_H

#include "tsumikoro_hal.h"
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief STM32-specific platform configuration
 *
 * This structure should be passed as platform_data in tsumikoro_hal_config_t
 */
typedef struct {
    USART_TypeDef *uart_instance;     /**< UART instance (e.g., USART1, USART2) */
    DMA_Channel_TypeDef *dma_tx;      /**< DMA channel for TX (optional, can be NULL) */
    DMA_Channel_TypeDef *dma_rx;      /**< DMA channel for RX (optional, can be NULL) */
    uint32_t dma_tx_request;          /**< DMA request for TX (e.g., DMA_REQUEST_USART1_TX) */
    uint32_t dma_rx_request;          /**< DMA request for RX (e.g., DMA_REQUEST_USART1_RX) */
    GPIO_TypeDef *de_port;            /**< GPIO port for DE (Driver Enable) pin */
    uint16_t de_pin;                  /**< GPIO pin for DE (Driver Enable) */
    GPIO_TypeDef *re_port;            /**< GPIO port for RE (Receive Enable) pin (optional) */
    uint16_t re_pin;                  /**< GPIO pin for RE (Receive Enable) (optional) */
    IRQn_Type uart_irq;               /**< UART IRQ number */
    IRQn_Type dma_tx_irq;             /**< DMA TX IRQ number (if DMA enabled) */
    IRQn_Type dma_rx_irq;             /**< DMA RX IRQ number (if DMA enabled) */
} tsumikoro_hal_stm32_config_t;

/**
 * @brief Initialize UART peripheral (must be called before HAL init)
 *
 * Configures GPIO, UART, and DMA peripherals. Should be called during
 * application initialization before calling tsumikoro_hal_init().
 *
 * @param config STM32-specific platform configuration
 * @param baud_rate Desired baud rate (e.g., 1000000 for 1Mbaud)
 * @return true on success, false on error
 */
bool tsumikoro_hal_stm32_init_peripheral(const tsumikoro_hal_stm32_config_t *config,
                                          uint32_t baud_rate);

/**
 * @brief UART IRQ handler (call from UART interrupt)
 *
 * Must be called from the UART interrupt handler (e.g., USART1_IRQHandler)
 *
 * @param handle HAL handle returned from tsumikoro_hal_init()
 */
void tsumikoro_hal_stm32_uart_irq_handler(tsumikoro_hal_handle_t handle);

/**
 * @brief DMA TX IRQ handler (call from DMA TX interrupt)
 *
 * Must be called from the DMA TX complete interrupt handler
 *
 * @param handle HAL handle returned from tsumikoro_hal_init()
 */
void tsumikoro_hal_stm32_dma_tx_irq_handler(tsumikoro_hal_handle_t handle);

/**
 * @brief DMA RX IRQ handler (call from DMA RX interrupt)
 *
 * Must be called from the DMA RX complete interrupt handler
 *
 * @param handle HAL handle returned from tsumikoro_hal_init()
 */
void tsumikoro_hal_stm32_dma_rx_irq_handler(tsumikoro_hal_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_HAL_STM32_H */
