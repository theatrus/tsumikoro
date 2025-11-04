/**
 * @file tsumikoro_hal_mock.h
 * @brief Mock HAL implementation for testing
 *
 * Simulates a multi-drop bus with multiple devices connected in memory.
 * Allows testing of controller-peripheral communication without hardware.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_HAL_MOCK_H
#define TSUMIKORO_HAL_MOCK_H

#include "tsumikoro_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of devices on mock bus
 */
#define TSUMIKORO_MOCK_MAX_DEVICES 16

/**
 * @brief Mock bus buffer size
 */
#define TSUMIKORO_MOCK_BUS_BUFFER_SIZE 256

/**
 * @brief Mock bus handle (shared between all devices)
 *
 * This is an opaque handle to the shared bus.
 * All devices on the same bus share this handle.
 */
typedef struct tsumikoro_mock_bus* tsumikoro_mock_bus_handle_t;

/**
 * @brief Create a mock bus
 *
 * Creates a virtual bus that multiple mock devices can connect to.
 *
 * @return Bus handle, or NULL on error
 */
tsumikoro_mock_bus_handle_t tsumikoro_mock_bus_create(void);

/**
 * @brief Destroy a mock bus
 *
 * Frees all resources associated with the bus.
 * All devices must be deinitialized first.
 *
 * @param bus Bus handle
 */
void tsumikoro_mock_bus_destroy(tsumikoro_mock_bus_handle_t bus);

/**
 * @brief Create a mock device connected to a bus
 *
 * This is a convenience wrapper around tsumikoro_hal_init()
 * that connects the device to a mock bus.
 *
 * @param bus Bus handle to connect to
 * @param config HAL configuration
 * @param rx_callback RX callback (optional)
 * @param user_data User data for callback
 * @return HAL handle, or NULL on error
 */
tsumikoro_hal_handle_t tsumikoro_mock_create_device(tsumikoro_mock_bus_handle_t bus,
                                                     const tsumikoro_hal_config_t *config,
                                                     tsumikoro_hal_rx_callback_t rx_callback,
                                                     void *user_data);

/**
 * @brief Process pending events on the bus
 *
 * Simulates time passing and delivers pending messages.
 * Should be called periodically in tests to process transmissions.
 *
 * @param bus Bus handle
 * @param time_advance_ms How much time to advance (for timeout simulation)
 */
void tsumikoro_mock_bus_process(tsumikoro_mock_bus_handle_t bus, uint32_t time_advance_ms);

/**
 * @brief Get number of bytes pending on bus
 *
 * For debugging and verification in tests.
 *
 * @param bus Bus handle
 * @return Number of bytes currently in transit
 */
size_t tsumikoro_mock_bus_get_pending_bytes(tsumikoro_mock_bus_handle_t bus);

/**
 * @brief Inject an error into the bus
 *
 * For testing error handling. Can corrupt data, drop packets, etc.
 *
 * @param bus Bus handle
 * @param error_type Type of error to inject
 */
typedef enum {
    TSUMIKORO_MOCK_ERROR_NONE = 0,       /**< No error */
    TSUMIKORO_MOCK_ERROR_CRC_CORRUPT,    /**< Corrupt CRC byte */
    TSUMIKORO_MOCK_ERROR_DROP_PACKET,    /**< Drop next packet */
    TSUMIKORO_MOCK_ERROR_TRUNCATE,       /**< Truncate next packet */
    TSUMIKORO_MOCK_ERROR_BIT_FLIP,       /**< Flip a random bit */
} tsumikoro_mock_error_type_t;

void tsumikoro_mock_bus_inject_error(tsumikoro_mock_bus_handle_t bus,
                                      tsumikoro_mock_error_type_t error_type);

/**
 * @brief Get statistics from mock bus
 *
 * For verification in tests.
 */
typedef struct {
    uint32_t packets_transmitted;   /**< Total packets sent */
    uint32_t packets_received;      /**< Total packets received */
    uint32_t packets_dropped;       /**< Packets dropped due to errors */
    uint32_t crc_errors_injected;   /**< CRC errors injected */
    uint32_t collisions_detected;   /**< Collisions detected */
} tsumikoro_mock_bus_stats_t;

void tsumikoro_mock_bus_get_stats(tsumikoro_mock_bus_handle_t bus,
                                   tsumikoro_mock_bus_stats_t *stats);

/**
 * @brief Reset mock bus statistics
 *
 * @param bus Bus handle
 */
void tsumikoro_mock_bus_reset_stats(tsumikoro_mock_bus_handle_t bus);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_HAL_MOCK_H */
