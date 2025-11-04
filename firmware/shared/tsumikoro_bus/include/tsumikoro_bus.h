/**
 * @file tsumikoro_bus.h
 * @brief High-level bus protocol handler with state machine
 *
 * Provides command queuing, automatic retry, timeout handling, and
 * response tracking for the Tsumikoro multi-drop bus protocol.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_BUS_H
#define TSUMIKORO_BUS_H

#include "tsumikoro_protocol.h"
#include "tsumikoro_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bus configuration
 */
typedef struct {
    uint32_t response_timeout_ms;   /**< Timeout waiting for response (default: 100ms) */
    uint32_t retry_count;            /**< Number of retries on timeout/error (default: 3) */
    uint32_t retry_delay_ms;         /**< Delay between retries (default: 10ms) */
    uint32_t bus_idle_timeout_ms;    /**< Bus idle detection timeout (default: 5ms) */
    bool auto_retry;                 /**< Automatically retry on timeout/NAK (default: true) */
} tsumikoro_bus_config_t;

/**
 * @brief Default bus configuration
 */
#define TSUMIKORO_BUS_DEFAULT_CONFIG() { \
    .response_timeout_ms = 100, \
    .retry_count = 3, \
    .retry_delay_ms = 10, \
    .bus_idle_timeout_ms = 5, \
    .auto_retry = true \
}

/**
 * @brief Bus state machine states
 */
typedef enum {
    TSUMIKORO_BUS_STATE_IDLE,              /**< Idle, ready to transmit */
    TSUMIKORO_BUS_STATE_WAIT_BUS_IDLE,     /**< Waiting for bus to become idle */
    TSUMIKORO_BUS_STATE_TRANSMITTING,      /**< Transmitting packet */
    TSUMIKORO_BUS_STATE_WAITING_RESPONSE,  /**< Waiting for response */
    TSUMIKORO_BUS_STATE_RETRY_DELAY,       /**< Delay before retry */
    TSUMIKORO_BUS_STATE_ERROR,             /**< Error state */
} tsumikoro_bus_state_t;

/**
 * @brief Command completion status
 */
typedef enum {
    TSUMIKORO_CMD_STATUS_PENDING,    /**< Command pending/in progress */
    TSUMIKORO_CMD_STATUS_SUCCESS,    /**< Command completed successfully */
    TSUMIKORO_CMD_STATUS_TIMEOUT,    /**< Command timed out */
    TSUMIKORO_CMD_STATUS_NAK,        /**< Command rejected (NAK) */
    TSUMIKORO_CMD_STATUS_CRC_ERROR,  /**< CRC error in response */
    TSUMIKORO_CMD_STATUS_FAILED,     /**< Command failed (other error) */
} tsumikoro_cmd_status_t;

/**
 * @brief Response callback function type
 *
 * Called when a response is received or command completes/fails.
 *
 * @param status Command completion status
 * @param response Response packet (NULL if timeout/error)
 * @param user_data User data pointer from send function
 */
typedef void (*tsumikoro_response_callback_t)(tsumikoro_cmd_status_t status,
                                               const tsumikoro_packet_t *response,
                                               void *user_data);

/**
 * @brief Unsolicited message callback function type
 *
 * Called when a message is received that doesn't match a pending command.
 * Used for broadcasts, async notifications, etc.
 *
 * @param packet Received packet
 * @param user_data User data pointer from bus init
 */
typedef void (*tsumikoro_unsolicited_callback_t)(const tsumikoro_packet_t *packet,
                                                  void *user_data);

/**
 * @brief Opaque bus handle
 */
typedef void* tsumikoro_bus_handle_t;

/**
 * @brief Initialize bus handler
 *
 * @param hal_handle HAL handle (already initialized)
 * @param config Bus configuration (or NULL for defaults)
 * @param unsolicited_callback Callback for unsolicited messages (optional)
 * @param user_data User data for unsolicited callback
 * @return Bus handle on success, NULL on error
 */
tsumikoro_bus_handle_t tsumikoro_bus_init(tsumikoro_hal_handle_t hal_handle,
                                           const tsumikoro_bus_config_t *config,
                                           tsumikoro_unsolicited_callback_t unsolicited_callback,
                                           void *user_data);

/**
 * @brief Deinitialize bus handler
 *
 * @param handle Bus handle
 */
void tsumikoro_bus_deinit(tsumikoro_bus_handle_t handle);

/**
 * @brief Send command and wait for response (blocking)
 *
 * Sends a command and blocks until response received or timeout.
 * Handles automatic retry if configured.
 *
 * @param handle Bus handle
 * @param packet Command packet to send
 * @param response Buffer for response packet (optional, can be NULL)
 * @param timeout_ms Timeout in milliseconds (0 = use bus default)
 * @return Command status
 */
tsumikoro_cmd_status_t tsumikoro_bus_send_command_blocking(tsumikoro_bus_handle_t handle,
                                                            const tsumikoro_packet_t *packet,
                                                            tsumikoro_packet_t *response,
                                                            uint32_t timeout_ms);

/**
 * @brief Send command asynchronously with callback
 *
 * Queues command for transmission. Callback will be invoked when
 * response received or command times out.
 *
 * @param handle Bus handle
 * @param packet Command packet to send
 * @param callback Response callback
 * @param user_data User data for callback
 * @return TSUMIKORO_STATUS_OK if queued, error otherwise
 */
tsumikoro_status_t tsumikoro_bus_send_command_async(tsumikoro_bus_handle_t handle,
                                                     const tsumikoro_packet_t *packet,
                                                     tsumikoro_response_callback_t callback,
                                                     void *user_data);

/**
 * @brief Send command without expecting response (fire-and-forget)
 *
 * Used for broadcasts, notifications, etc.
 *
 * @param handle Bus handle
 * @param packet Command packet to send
 * @return TSUMIKORO_STATUS_OK if sent, error otherwise
 */
tsumikoro_status_t tsumikoro_bus_send_no_response(tsumikoro_bus_handle_t handle,
                                                   const tsumikoro_packet_t *packet);

/**
 * @brief Process bus state machine
 *
 * Must be called periodically (e.g., every 1ms) to process
 * timeouts, retries, and state transitions.
 *
 * @param handle Bus handle
 */
void tsumikoro_bus_process(tsumikoro_bus_handle_t handle);

/**
 * @brief Get current bus state
 *
 * For debugging and diagnostics.
 *
 * @param handle Bus handle
 * @return Current bus state
 */
tsumikoro_bus_state_t tsumikoro_bus_get_state(tsumikoro_bus_handle_t handle);

/**
 * @brief Check if bus is idle
 *
 * @param handle Bus handle
 * @return true if bus is idle and ready to send
 */
bool tsumikoro_bus_is_idle(tsumikoro_bus_handle_t handle);

/**
 * @brief Cancel pending command
 *
 * Cancels the currently pending command (if any).
 * Callback will be invoked with TSUMIKORO_CMD_STATUS_FAILED.
 *
 * @param handle Bus handle
 */
void tsumikoro_bus_cancel_pending(tsumikoro_bus_handle_t handle);

/**
 * @brief Get bus statistics
 *
 * For debugging and diagnostics.
 * Fields are volatile as they may be updated from ISR context.
 */
typedef struct {
    volatile uint32_t commands_sent;         /**< Total commands sent */
    volatile uint32_t responses_received;    /**< Total responses received */
    volatile uint32_t timeouts;              /**< Total timeouts */
    volatile uint32_t retries;               /**< Total retries attempted */
    volatile uint32_t crc_errors;            /**< Total CRC errors */
    volatile uint32_t unsolicited_messages;  /**< Total unsolicited messages */
} tsumikoro_bus_stats_t;

/**
 * @brief Get bus statistics
 *
 * @param handle Bus handle
 * @param stats Pointer to stats structure
 */
void tsumikoro_bus_get_stats(tsumikoro_bus_handle_t handle,
                              tsumikoro_bus_stats_t *stats);

/**
 * @brief Reset bus statistics
 *
 * @param handle Bus handle
 */
void tsumikoro_bus_reset_stats(tsumikoro_bus_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_BUS_H */
