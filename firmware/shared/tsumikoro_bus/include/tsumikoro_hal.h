/**
 * @file tsumikoro_hal.h
 * @brief Hardware Abstraction Layer interface for Tsumikoro bus
 *
 * Defines platform-independent HAL interface for UART communication.
 * Platform-specific implementations should be provided in:
 * - hal/stm32/    - STM32 HAL UART + DMA
 * - hal/esp32/    - ESP32 IDF UART driver
 * - hal/host/     - POSIX termios
 * - hal/mock/     - Mock HAL for testing
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_HAL_H
#define TSUMIKORO_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HAL status codes
 */
typedef enum {
    TSUMIKORO_HAL_OK = 0,          /**< Success */
    TSUMIKORO_HAL_ERROR,           /**< Generic error */
    TSUMIKORO_HAL_TIMEOUT,         /**< Operation timeout */
    TSUMIKORO_HAL_BUSY,            /**< Hardware busy */
    TSUMIKORO_HAL_NOT_READY,       /**< Hardware not ready */
} tsumikoro_hal_status_t;

/**
 * @brief Opaque HAL handle
 *
 * Platform-specific implementation contains actual hardware context
 */
typedef void* tsumikoro_hal_handle_t;

/**
 * @brief HAL configuration structure
 */
typedef struct {
    uint32_t baud_rate;            /**< Baud rate (e.g., 1000000 for 1Mbaud) */
    uint8_t device_id;             /**< This device's ID */
    bool is_controller;            /**< True if controller, false if peripheral */
    uint32_t turnaround_delay_bytes; /**< Bus turnaround delay in byte intervals (0 = no delay) */
    void *platform_data;           /**< Platform-specific configuration */
} tsumikoro_hal_config_t;

/**
 * @brief RX callback function type
 *
 * Called when data is received. Should copy data quickly and return.
 *
 * @param data Received data buffer
 * @param len Number of bytes received
 * @param user_data User data pointer from init
 */
typedef void (*tsumikoro_hal_rx_callback_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Initialize HAL
 *
 * @param config Configuration structure
 * @param rx_callback Callback for received data (optional, can be NULL)
 * @param user_data User data to pass to callbacks
 * @return HAL handle on success, NULL on error
 */
tsumikoro_hal_handle_t tsumikoro_hal_init(const tsumikoro_hal_config_t *config,
                                           tsumikoro_hal_rx_callback_t rx_callback,
                                           void *user_data);

/**
 * @brief Deinitialize HAL
 *
 * @param handle HAL handle
 */
void tsumikoro_hal_deinit(tsumikoro_hal_handle_t handle);

/**
 * @brief Transmit data
 *
 * Blocking or non-blocking depending on platform implementation.
 * For DMA implementations, may return immediately and transmit in background.
 *
 * @param handle HAL handle
 * @param data Data to transmit
 * @param len Number of bytes to transmit
 * @return Status code
 */
tsumikoro_hal_status_t tsumikoro_hal_transmit(tsumikoro_hal_handle_t handle,
                                               const uint8_t *data,
                                               size_t len);

/**
 * @brief Receive data (polling mode)
 *
 * Used if RX callback is not registered.
 * Blocks until data is available or timeout expires.
 *
 * @param handle HAL handle
 * @param buffer Buffer for received data
 * @param max_len Maximum bytes to receive
 * @param received_len Pointer to store actual bytes received
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
 * @return Status code
 */
tsumikoro_hal_status_t tsumikoro_hal_receive(tsumikoro_hal_handle_t handle,
                                              uint8_t *buffer,
                                              size_t max_len,
                                              size_t *received_len,
                                              uint32_t timeout_ms);

/**
 * @brief Check if bus is idle (no carrier detected)
 *
 * Used for CSMA/CD collision avoidance.
 *
 * @param handle HAL handle
 * @return true if bus is idle, false if active
 */
bool tsumikoro_hal_is_bus_idle(tsumikoro_hal_handle_t handle);

/**
 * @brief Enable transmitter (for half-duplex control)
 *
 * Asserts DE (Driver Enable) signal on RS-485 transceivers.
 * Some platforms may handle this automatically.
 *
 * @param handle HAL handle
 */
void tsumikoro_hal_enable_tx(tsumikoro_hal_handle_t handle);

/**
 * @brief Disable transmitter (return to receive mode)
 *
 * De-asserts DE signal on RS-485 transceivers.
 *
 * @param handle HAL handle
 */
void tsumikoro_hal_disable_tx(tsumikoro_hal_handle_t handle);

/**
 * @brief Delay for specified microseconds
 *
 * Platform-specific implementation (busy-wait, timer, etc.)
 *
 * @param us Microseconds to delay
 */
void tsumikoro_hal_delay_us(uint32_t us);

/**
 * @brief Get monotonic timestamp in milliseconds
 *
 * Used for timeout tracking. Should not wrap frequently.
 *
 * @return Timestamp in milliseconds
 */
uint32_t tsumikoro_hal_get_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_HAL_H */
