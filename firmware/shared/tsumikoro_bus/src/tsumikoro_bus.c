/**
 * @file tsumikoro_bus.c
 * @brief High-level bus protocol handler implementation
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_bus.h"
#include "tsumikoro_crc8.h"
#include <stdlib.h>
#include <string.h>

/* Debug logging (define TSUMIKORO_BUS_DEBUG to enable) */
#ifdef TSUMIKORO_BUS_DEBUG
#include <stdio.h>
#define BUS_DEBUG(...) printf("[BUS] " __VA_ARGS__)
#else
#define BUS_DEBUG(...) ((void)0)
#endif

/**
 * @brief Pending command structure
 *
 * Fields marked volatile are accessed from both main context and ISR context
 * (via callbacks invoked from bus_complete_command which may run in ISR).
 */
typedef struct {
    tsumikoro_packet_t packet;           /**< Command packet */
    tsumikoro_response_callback_t callback;  /**< Response callback */
    void *user_data;                     /**< User data for callback */
    uint32_t send_time_ms;               /**< Time command was sent */
    volatile uint32_t retry_count;       /**< Retries attempted (volatile: read in ISR) */
    volatile bool active;                /**< Command is active (volatile: modified in ISR) */
    bool expects_response;               /**< Expecting a response */
} tsumikoro_pending_cmd_t;

/**
 * @brief Bus handler internal state
 */
typedef struct {
    tsumikoro_hal_handle_t hal;          /**< HAL handle */
    tsumikoro_bus_config_t config;       /**< Bus configuration */
    volatile tsumikoro_bus_state_t state; /**< Current state (volatile: may be read in ISR) */

    // Callbacks
    tsumikoro_unsolicited_callback_t unsolicited_callback;
    void *unsolicited_user_data;

    // Pending command
    tsumikoro_pending_cmd_t pending_cmd;

    // Last transmitted packet (for echo detection/collision detection)
    uint8_t last_tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t last_tx_len;
    volatile bool expecting_tx_echo;     /**< Expecting to receive our own transmission (volatile: read in ISR) */

    // RX buffer for incoming packets (volatile: written by HAL RX callback in ISR context)
    volatile uint8_t rx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    volatile size_t rx_buffer_len;

    // Timing
    uint32_t state_enter_time_ms;        /**< Time current state was entered */
    uint32_t last_process_time_ms;       /**< Last process() call time */

    // Statistics
    tsumikoro_bus_stats_t stats;
} tsumikoro_bus_t;

/* ========== Internal Helper Functions ========== */

/**
 * @brief Change bus state
 */
static void bus_set_state(tsumikoro_bus_t *bus, tsumikoro_bus_state_t new_state)
{
#ifdef TSUMIKORO_BUS_DEBUG
    const char *state_names[] = {
        "IDLE", "WAIT_BUS_IDLE", "TRANSMITTING",
        "WAITING_RESPONSE", "RETRY_DELAY", "ERROR"
    };
    BUS_DEBUG("State: %s -> %s\n",
              state_names[bus->state],
              state_names[new_state]);
#endif
    bus->state = new_state;
    bus->state_enter_time_ms = tsumikoro_hal_get_time_ms();
}

/**
 * @brief Get elapsed time in current state
 */
static uint32_t bus_get_state_elapsed_ms(tsumikoro_bus_t *bus)
{
    uint32_t now = tsumikoro_hal_get_time_ms();
    return now - bus->state_enter_time_ms;
}

/**
 * @brief Complete pending command
 */
static void bus_complete_command(tsumikoro_bus_t *bus,
                                  tsumikoro_cmd_status_t status,
                                  const tsumikoro_packet_t *response)
{
    if (!bus->pending_cmd.active) {
        return;
    }

    BUS_DEBUG("Command complete: cmd=0x%04X, status=%d, retry=%u\n",
              bus->pending_cmd.packet.command, status, bus->pending_cmd.retry_count);

    // Update statistics
    if (status == TSUMIKORO_CMD_STATUS_SUCCESS) {
        bus->stats.responses_received++;
    } else if (status == TSUMIKORO_CMD_STATUS_TIMEOUT) {
        bus->stats.timeouts++;
    } else if (status == TSUMIKORO_CMD_STATUS_CRC_ERROR) {
        bus->stats.crc_errors++;
    }

    // Invoke callback if registered
    if (bus->pending_cmd.callback) {
        bus->pending_cmd.callback(status, response, bus->pending_cmd.user_data);
    }

    // Clear pending command
    bus->pending_cmd.active = false;

    // Return to idle
    bus_set_state(bus, TSUMIKORO_BUS_STATE_IDLE);
}

/**
 * @brief Transmit pending command
 */
static tsumikoro_status_t bus_transmit_pending(tsumikoro_bus_t *bus)
{
    if (!bus->pending_cmd.active) {
        return TSUMIKORO_STATUS_ERROR;
    }

    BUS_DEBUG("Transmitting cmd=0x%04X to device=0x%02X\n",
              bus->pending_cmd.packet.command,
              bus->pending_cmd.packet.device_id);

    // Encode packet
    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&bus->pending_cmd.packet,
                                             tx_buffer, sizeof(tx_buffer));
    if (tx_len == 0) {
        BUS_DEBUG("Encode failed\n");
        return TSUMIKORO_STATUS_ERROR;
    }

    // Store transmitted packet for echo detection (RS-485 receives own transmission)
    memcpy(bus->last_tx_buffer, tx_buffer, tx_len);
    bus->last_tx_len = tx_len;
    bus->expecting_tx_echo = true;

    // Transmit via HAL
    tsumikoro_hal_enable_tx(bus->hal);
    tsumikoro_hal_status_t hal_status = tsumikoro_hal_transmit(bus->hal, tx_buffer, tx_len);
    tsumikoro_hal_disable_tx(bus->hal);

    if (hal_status != TSUMIKORO_HAL_OK) {
        BUS_DEBUG("Transmit failed: hal_status=%d\n", hal_status);
        bus->expecting_tx_echo = false;
        return TSUMIKORO_STATUS_ERROR;
    }

    // Update state
    bus->pending_cmd.send_time_ms = tsumikoro_hal_get_time_ms();
    bus->stats.commands_sent++;

    if (bus->pending_cmd.expects_response) {
        bus_set_state(bus, TSUMIKORO_BUS_STATE_WAITING_RESPONSE);
    } else {
        // No response expected, complete immediately
        bus_complete_command(bus, TSUMIKORO_CMD_STATUS_SUCCESS, NULL);
    }

    return TSUMIKORO_STATUS_OK;
}

/**
 * @brief Retry pending command
 */
static void bus_retry_command(tsumikoro_bus_t *bus)
{
    if (!bus->pending_cmd.active) {
        return;
    }

    bus->pending_cmd.retry_count++;
    bus->stats.retries++;

    BUS_DEBUG("Retry %u/%u for cmd=0x%04X\n",
              bus->pending_cmd.retry_count, bus->config.retry_count,
              bus->pending_cmd.packet.command);

    if (bus->pending_cmd.retry_count >= bus->config.retry_count) {
        // Max retries exceeded
        BUS_DEBUG("Max retries exceeded\n");
        bus_complete_command(bus, TSUMIKORO_CMD_STATUS_TIMEOUT, NULL);
    } else {
        // Enter retry delay state
        bus_set_state(bus, TSUMIKORO_BUS_STATE_RETRY_DELAY);
    }
}

/**
 * @brief Process received packet
 */
static void bus_process_rx_packet(tsumikoro_bus_t *bus)
{
    // Poll HAL for received data (if using polling mode)
    // Use local buffer to avoid volatile qualifier issues
    uint8_t local_rx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t local_rx_len = 0;

    if (bus->rx_buffer_len == 0) {
        size_t received_len = 0;
        tsumikoro_hal_status_t hal_status = tsumikoro_hal_receive(
            bus->hal, local_rx_buffer, sizeof(local_rx_buffer), &received_len, 0);

        if (hal_status == TSUMIKORO_HAL_OK && received_len > 0) {
            // Copy to volatile buffer (memory barrier)
            for (size_t i = 0; i < received_len; i++) {
                bus->rx_buffer[i] = local_rx_buffer[i];
            }
            bus->rx_buffer_len = received_len;
        }
    }

    // Snapshot volatile data to local variables (memory barrier)
    local_rx_len = bus->rx_buffer_len;
    if (local_rx_len == 0) {
        return;
    }

    // Copy volatile buffer to local buffer
    for (size_t i = 0; i < local_rx_len; i++) {
        local_rx_buffer[i] = bus->rx_buffer[i];
    }

    BUS_DEBUG("RX packet: %zu bytes\n", local_rx_len);

    // Check if this is our own transmission echo (RS-485 with RE asserted)
    if (bus->expecting_tx_echo &&
        local_rx_len == bus->last_tx_len &&
        memcmp(local_rx_buffer, bus->last_tx_buffer, local_rx_len) == 0) {
        BUS_DEBUG("Ignoring TX echo\n");
        bus->expecting_tx_echo = false;
        bus->rx_buffer_len = 0;
        return;
    }

    // If we expected an echo but got something different, that's a collision
    if (bus->expecting_tx_echo) {
        BUS_DEBUG("Collision detected! Expected echo but received different packet\n");
        bus->expecting_tx_echo = false;
        // TODO: Handle collision (retry transmission)
    }

    // Decode packet (using local non-volatile buffer)
    tsumikoro_packet_t rx_packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(local_rx_buffer,
                                                         local_rx_len,
                                                         &rx_packet);

    // Clear volatile RX buffer
    bus->rx_buffer_len = 0;

    if (status == TSUMIKORO_STATUS_CRC_ERROR) {
        BUS_DEBUG("CRC error\n");
        // CRC error - if we're waiting for response, this might be our response
        if (bus->state == TSUMIKORO_BUS_STATE_WAITING_RESPONSE) {
            if (bus->config.auto_retry) {
                bus_retry_command(bus);
            } else {
                bus_complete_command(bus, TSUMIKORO_CMD_STATUS_CRC_ERROR, NULL);
            }
        }
        return;
    }

    if (status != TSUMIKORO_STATUS_OK) {
        BUS_DEBUG("Decode failed: status=%d\n", status);
        // Invalid packet - ignore
        return;
    }

    BUS_DEBUG("RX decoded: cmd=0x%04X, device=0x%02X, data_len=%u\n",
              rx_packet.command, rx_packet.device_id, rx_packet.data_len);

    // Check if this is a response to our pending command
    if (bus->pending_cmd.active && bus->pending_cmd.expects_response) {
        // Simple matching: check if device_id matches our device ID
        // (response should be addressed to us)
        // In a more sophisticated implementation, could match command ID, sequence number, etc.

        // For now, if we're waiting for a response, assume any valid packet is the response
        if (bus->state == TSUMIKORO_BUS_STATE_WAITING_RESPONSE) {
            // Check for NAK
            if (rx_packet.data_len > 0 && rx_packet.data[0] == TSUMIKORO_STATUS_NAK) {
                BUS_DEBUG("Received NAK\n");
                if (bus->config.auto_retry) {
                    bus_retry_command(bus);
                } else {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_NAK, &rx_packet);
                }
            } else {
                // Success
                BUS_DEBUG("Response matched pending command\n");
                bus_complete_command(bus, TSUMIKORO_CMD_STATUS_SUCCESS, &rx_packet);
            }
            return;
        }
    }

    // Unsolicited message
    BUS_DEBUG("Unsolicited message\n");
    bus->stats.unsolicited_messages++;
    if (bus->unsolicited_callback) {
        bus->unsolicited_callback(&rx_packet, bus->unsolicited_user_data);
    }
}

/* ========== State Machine ========== */

/**
 * @brief Process state machine
 */
static void bus_process_state_machine(tsumikoro_bus_t *bus)
{
    // Check for received data
    bus_process_rx_packet(bus);

    // Process current state
    switch (bus->state) {
        case TSUMIKORO_BUS_STATE_IDLE:
            // Nothing to do - wait for command
            break;

        case TSUMIKORO_BUS_STATE_WAIT_BUS_IDLE:
            // Wait for bus to become idle
            if (tsumikoro_hal_is_bus_idle(bus->hal)) {
                // Bus is idle, transmit
                if (bus_transmit_pending(bus) != TSUMIKORO_STATUS_OK) {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_FAILED, NULL);
                }
            } else if (bus_get_state_elapsed_ms(bus) >= bus->config.bus_idle_timeout_ms) {
                // Timeout waiting for bus idle
                if (bus->config.auto_retry) {
                    bus_retry_command(bus);
                } else {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_TIMEOUT, NULL);
                }
            }
            break;

        case TSUMIKORO_BUS_STATE_TRANSMITTING:
            // HAL handles transmission - should transition quickly
            break;

        case TSUMIKORO_BUS_STATE_WAITING_RESPONSE:
            // Check for timeout
            if (bus_get_state_elapsed_ms(bus) >= bus->config.response_timeout_ms) {
                if (bus->config.auto_retry) {
                    bus_retry_command(bus);
                } else {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_TIMEOUT, NULL);
                }
            }
            break;

        case TSUMIKORO_BUS_STATE_RETRY_DELAY:
            // Wait for retry delay to expire
            if (bus_get_state_elapsed_ms(bus) >= bus->config.retry_delay_ms) {
                // Retry delay expired, try again
                bus_set_state(bus, TSUMIKORO_BUS_STATE_WAIT_BUS_IDLE);
            }
            break;

        case TSUMIKORO_BUS_STATE_ERROR:
            // Error state - should not get here
            bus_set_state(bus, TSUMIKORO_BUS_STATE_IDLE);
            break;
    }
}

/* ========== Public API ========== */

tsumikoro_bus_handle_t tsumikoro_bus_init(tsumikoro_hal_handle_t hal_handle,
                                           const tsumikoro_bus_config_t *config,
                                           tsumikoro_unsolicited_callback_t unsolicited_callback,
                                           void *user_data)
{
    if (hal_handle == NULL) {
        return NULL;
    }

    // Allocate bus structure
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)calloc(1, sizeof(tsumikoro_bus_t));
    if (bus == NULL) {
        return NULL;
    }

    // Initialize
    bus->hal = hal_handle;
    bus->state = TSUMIKORO_BUS_STATE_IDLE;
    bus->unsolicited_callback = unsolicited_callback;
    bus->unsolicited_user_data = user_data;

    // Copy config or use defaults
    if (config) {
        bus->config = *config;
    } else {
        tsumikoro_bus_config_t default_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
        bus->config = default_config;
    }

    bus->state_enter_time_ms = tsumikoro_hal_get_time_ms();
    bus->last_process_time_ms = bus->state_enter_time_ms;

    return (tsumikoro_bus_handle_t)bus;
}

void tsumikoro_bus_deinit(tsumikoro_bus_handle_t handle)
{
    if (handle) {
        free(handle);
    }
}

tsumikoro_status_t tsumikoro_bus_send_command_async(tsumikoro_bus_handle_t handle,
                                                     const tsumikoro_packet_t *packet,
                                                     tsumikoro_response_callback_t callback,
                                                     void *user_data)
{
    if (handle == NULL || packet == NULL) {
        return TSUMIKORO_STATUS_ERROR;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;

    // Check if already busy
    if (bus->pending_cmd.active) {
        return TSUMIKORO_STATUS_BUSY;
    }

    // Queue command
    bus->pending_cmd.packet = *packet;
    bus->pending_cmd.callback = callback;
    bus->pending_cmd.user_data = user_data;
    bus->pending_cmd.retry_count = 0;
    bus->pending_cmd.active = true;
    bus->pending_cmd.expects_response = true;

    // Start transmission process
    bus_set_state(bus, TSUMIKORO_BUS_STATE_WAIT_BUS_IDLE);

    return TSUMIKORO_STATUS_OK;
}

tsumikoro_status_t tsumikoro_bus_send_no_response(tsumikoro_bus_handle_t handle,
                                                   const tsumikoro_packet_t *packet)
{
    if (handle == NULL || packet == NULL) {
        return TSUMIKORO_STATUS_ERROR;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;

    // Check if already busy
    if (bus->pending_cmd.active) {
        return TSUMIKORO_STATUS_BUSY;
    }

    // Queue command
    bus->pending_cmd.packet = *packet;
    bus->pending_cmd.callback = NULL;
    bus->pending_cmd.user_data = NULL;
    bus->pending_cmd.retry_count = 0;
    bus->pending_cmd.active = true;
    bus->pending_cmd.expects_response = false;

    // Start transmission process
    bus_set_state(bus, TSUMIKORO_BUS_STATE_WAIT_BUS_IDLE);

    return TSUMIKORO_STATUS_OK;
}

tsumikoro_cmd_status_t tsumikoro_bus_send_command_blocking(tsumikoro_bus_handle_t handle,
                                                            const tsumikoro_packet_t *packet,
                                                            tsumikoro_packet_t *response,
                                                            uint32_t timeout_ms)
{
    if (handle == NULL || packet == NULL) {
        return TSUMIKORO_CMD_STATUS_FAILED;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;

    // Use bus default timeout if not specified
    if (timeout_ms == 0) {
        timeout_ms = bus->config.response_timeout_ms;
    }

    // Send command asynchronously
    volatile tsumikoro_cmd_status_t result = TSUMIKORO_CMD_STATUS_PENDING;
    volatile tsumikoro_packet_t temp_response;
    volatile bool done = false;

    // Callback to capture result
    void blocking_callback(tsumikoro_cmd_status_t status,
                          const tsumikoro_packet_t *resp,
                          void *user_data)
    {
        (void)user_data;
        result = status;
        if (resp && response) {
            temp_response = *resp;
        }
        done = true;
    }

    tsumikoro_status_t send_status = tsumikoro_bus_send_command_async(handle, packet,
                                                                       blocking_callback, NULL);
    if (send_status != TSUMIKORO_STATUS_OK) {
        return TSUMIKORO_CMD_STATUS_FAILED;
    }

    // Wait for completion
    uint32_t start_time = tsumikoro_hal_get_time_ms();
    while (!done) {
        tsumikoro_bus_process(handle);

        // Check for overall timeout
        uint32_t elapsed = tsumikoro_hal_get_time_ms() - start_time;
        if (elapsed >= (timeout_ms * (bus->config.retry_count + 1))) {
            bus_complete_command(bus, TSUMIKORO_CMD_STATUS_TIMEOUT, NULL);
            return TSUMIKORO_CMD_STATUS_TIMEOUT;
        }

        // Small delay to avoid busy-wait
        tsumikoro_hal_delay_us(100);
    }

    // Copy response if requested
    if (response && result == TSUMIKORO_CMD_STATUS_SUCCESS) {
        *response = (tsumikoro_packet_t)temp_response;
    }

    return result;
}

void tsumikoro_bus_process(tsumikoro_bus_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    bus->last_process_time_ms = tsumikoro_hal_get_time_ms();

    bus_process_state_machine(bus);
}

tsumikoro_bus_state_t tsumikoro_bus_get_state(tsumikoro_bus_handle_t handle)
{
    if (handle == NULL) {
        return TSUMIKORO_BUS_STATE_ERROR;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    return bus->state;
}

bool tsumikoro_bus_is_idle(tsumikoro_bus_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    return bus->state == TSUMIKORO_BUS_STATE_IDLE && !bus->pending_cmd.active;
}

void tsumikoro_bus_cancel_pending(tsumikoro_bus_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;

    if (bus->pending_cmd.active) {
        bus_complete_command(bus, TSUMIKORO_CMD_STATUS_FAILED, NULL);
    }
}

void tsumikoro_bus_get_stats(tsumikoro_bus_handle_t handle,
                              tsumikoro_bus_stats_t *stats)
{
    if (handle == NULL || stats == NULL) {
        return;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    *stats = bus->stats;
}

void tsumikoro_bus_reset_stats(tsumikoro_bus_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    memset(&bus->stats, 0, sizeof(bus->stats));
}
