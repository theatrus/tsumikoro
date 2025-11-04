/**
 * @file tsumikoro_hal_esp32.h
 * @brief ESP32 HAL implementation for Tsumikoro bus
 *
 * Implements the Tsumikoro HAL interface using ESP-IDF UART driver.
 * Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C3 series.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_HAL_ESP32_H
#define TSUMIKORO_HAL_ESP32_H

#include "tsumikoro_hal.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP32-specific platform configuration
 *
 * This structure should be passed as platform_data in tsumikoro_hal_config_t
 */
typedef struct {
    uart_port_t uart_port;          /**< UART port number (e.g., UART_NUM_1, UART_NUM_2) */
    int tx_pin;                      /**< GPIO pin for UART TX */
    int rx_pin;                      /**< GPIO pin for UART RX */
    int rts_pin;                     /**< GPIO pin for RTS/DE (RS-485 driver enable), or -1 if not used */
    int cts_pin;                     /**< GPIO pin for CTS/RE (RS-485 receiver enable), or -1 if not used */
    size_t rx_buffer_size;           /**< RX buffer size (default: 1024) */
    size_t tx_buffer_size;           /**< TX buffer size (default: 1024, 0 = no TX buffer/blocking) */
    bool use_rs485_mode;             /**< Enable RS-485 half-duplex mode */
} tsumikoro_hal_esp32_config_t;

/**
 * @brief Initialize UART peripheral (must be called before HAL init)
 *
 * Configures GPIO, UART, and installs UART driver with buffers.
 * Should be called during application initialization before calling tsumikoro_hal_init().
 *
 * @param config ESP32-specific platform configuration
 * @param baud_rate Desired baud rate (e.g., 1000000 for 1Mbaud)
 * @return true on success, false on error
 */
bool tsumikoro_hal_esp32_init_peripheral(const tsumikoro_hal_esp32_config_t *config,
                                          uint32_t baud_rate);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_HAL_ESP32_H */
