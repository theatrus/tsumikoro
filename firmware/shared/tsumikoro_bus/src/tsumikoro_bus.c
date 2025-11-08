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

#if TSUMIKORO_BUS_USE_RTOS
#include "tsumikoro_rtos.h"
#endif

/* Debug logging */
#ifdef ESP_PLATFORM
    // ESP32: Use ESP-IDF logging (always enabled for debugging)
    #include "esp_log.h"
    static const char *BUS_TAG = "tsumikoro_bus";
    #define BUS_DEBUG(fmt, ...) ESP_LOGI(BUS_TAG, fmt, ##__VA_ARGS__)
    #warning "Compiling tsumikoro_bus.c with ESP_PLATFORM defined - debug enabled"
#else
    // STM32/other: Use printf (controlled by TSUMIKORO_BUS_DEBUG)
    #ifdef TSUMIKORO_BUS_DEBUG
        #include <stdio.h>
        #define BUS_DEBUG(...) printf("[BUS] " __VA_ARGS__)
    #else
        #define BUS_DEBUG(...) ((void)0)
    #endif
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
 * @brief Context structure for blocking command callback
 *
 * Used to capture async callback results for blocking API.
 * Avoids GNU C nested function extension for C11 compatibility.
 */
typedef struct {
    volatile tsumikoro_cmd_status_t *result;      /**< Result status pointer */
    volatile tsumikoro_packet_t *temp_response;   /**< Temporary response storage */
    volatile bool *done;                          /**< Completion flag */
    tsumikoro_packet_t *response;                 /**< User's response buffer (may be NULL) */
} tsumikoro_blocking_callback_ctx_t;

#if TSUMIKORO_BUS_USE_RTOS
/**
 * @brief RX message for RX thread queue
 */
typedef struct {
    uint8_t data[TSUMIKORO_MAX_PACKET_LEN];  /**< Received data */
    size_t len;                               /**< Number of bytes */
} tsumikoro_rx_msg_t;

/**
 * @brief TX request message for TX thread queue
 */
typedef struct {
    uint8_t data[TSUMIKORO_MAX_PACKET_LEN];  /**< Data to transmit */
    size_t len;                               /**< Number of bytes */
} tsumikoro_tx_msg_t;

/**
 * @brief Handler event message for handler thread
 */
typedef enum {
    TSUMIKORO_HANDLER_EVENT_RX_PACKET,      /**< RX packet decoded */
    TSUMIKORO_HANDLER_EVENT_TX_COMPLETE,    /**< TX completed */
    TSUMIKORO_HANDLER_EVENT_TIMEOUT,        /**< Timeout occurred */
} tsumikoro_handler_event_type_t;

typedef struct {
    tsumikoro_handler_event_type_t type;     /**< Event type */
    tsumikoro_packet_t packet;                /**< Associated packet (if applicable) */
} tsumikoro_handler_event_t;
#endif

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

#if TSUMIKORO_BUS_USE_RTOS
    // RTOS resources
    tsumikoro_queue_t rx_queue;          /**< RX data queue (ISR -> RX thread) */
    tsumikoro_queue_t tx_queue;          /**< TX request queue (Handler -> TX thread) */
    tsumikoro_queue_t handler_queue;     /**< Handler event queue (RX/TX threads -> Handler thread) */
    tsumikoro_thread_t rx_thread;        /**< RX processing thread */
    tsumikoro_thread_t tx_thread;        /**< TX processing thread */
    tsumikoro_thread_t handler_thread;   /**< Command handler thread */
    tsumikoro_semaphore_t blocking_sem;  /**< Semaphore for blocking commands */
#endif
} tsumikoro_bus_t;

/* ========== Internal Helper Functions ========== */

/**
 * @brief Blocking callback for synchronous command execution
 *
 * Standard C11-compliant callback that uses context structure instead of
 * GNU C nested functions.
 */
static void bus_blocking_callback(tsumikoro_cmd_status_t status,
                                  const tsumikoro_packet_t *resp,
                                  void *user_data)
{
    tsumikoro_blocking_callback_ctx_t *ctx = (tsumikoro_blocking_callback_ctx_t *)user_data;

    *(ctx->result) = status;
    if (resp && ctx->response) {
        *(ctx->temp_response) = *resp;
    }
    *(ctx->done) = true;
}

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

#if TSUMIKORO_BUS_USE_RTOS
    // In RTOS mode, post to TX thread queue
    tsumikoro_tx_msg_t tx_msg;
    memcpy(tx_msg.data, tx_buffer, tx_len);
    tx_msg.len = tx_len;

    if (!tsumikoro_queue_send(bus->tx_queue, &tx_msg, 100)) {
        BUS_DEBUG("TX queue send failed\n");
        return TSUMIKORO_STATUS_ERROR;
    }

    // Update state
    bus->pending_cmd.send_time_ms = tsumikoro_hal_get_time_ms();
    bus->stats.commands_sent++;
    bus_set_state(bus, TSUMIKORO_BUS_STATE_TRANSMITTING);

#else
    // Bare-metal mode: transmit directly via HAL

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
#endif

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

    BUS_DEBUG("RX packet: %u bytes\n", (unsigned int)local_rx_len);

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
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(local_rx_buffer,
                                                         local_rx_len,
                                                         &rx_packet,
                                                         &bytes_consumed);

    // Remove consumed bytes from buffer
    if (status == TSUMIKORO_STATUS_OK && bytes_consumed > 0) {
        // Successfully decoded - remove consumed bytes, keep remaining data
        size_t remaining = (bytes_consumed < local_rx_len) ? (local_rx_len - bytes_consumed) : 0;

        if (remaining > 0) {
            // Move remaining bytes to start of buffer
            for (size_t i = 0; i < remaining; i++) {
                bus->rx_buffer[i] = bus->rx_buffer[bytes_consumed + i];
            }
        }
        bus->rx_buffer_len = remaining;
    } else {
        // Decode failed - clear buffer to avoid reprocessing
        bus->rx_buffer_len = 0;
    }

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
                BUS_DEBUG("Timeout waiting for bus idle (%ums elapsed)\n", bus_get_state_elapsed_ms(bus));
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
                BUS_DEBUG("Response timeout (%ums elapsed, expected response from 0x%02X)\n",
                         bus_get_state_elapsed_ms(bus), bus->pending_cmd.packet.device_id);
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

#if TSUMIKORO_BUS_USE_RTOS
/* ========== RTOS Implementation ========== */

/**
 * @brief HAL RX callback for RTOS mode (called from ISR)
 *
 * Posts received data to RX queue for processing by RX thread.
 */
static void bus_hal_rx_callback(const uint8_t *data, size_t len, void *user_data)
{
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)user_data;

    if (!bus || !data || len == 0 || len > TSUMIKORO_MAX_PACKET_LEN) {
        return;
    }

    // Create RX message
    tsumikoro_rx_msg_t rx_msg;
    memcpy(rx_msg.data, data, len);
    rx_msg.len = len;

    // Post to RX queue (non-blocking from ISR)
    tsumikoro_queue_send_from_isr(bus->rx_queue, &rx_msg);
}

/**
 * @brief RX thread function
 *
 * Waits for RX data from ISR, decodes packets, and posts events to handler thread.
 */
static void bus_rx_thread(void *arg)
{
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)arg;

    BUS_DEBUG("RX thread started\n");

    while (1) {
        tsumikoro_rx_msg_t rx_msg;

        // Block waiting for RX data
        if (!tsumikoro_queue_receive(bus->rx_queue, &rx_msg, UINT32_MAX)) {
            continue;
        }

        BUS_DEBUG("RX thread: received %u bytes\n", (unsigned int)rx_msg.len);

        // Check for TX echo
        if (bus->expecting_tx_echo &&
            rx_msg.len == bus->last_tx_len &&
            memcmp(rx_msg.data, bus->last_tx_buffer, rx_msg.len) == 0) {
            BUS_DEBUG("RX thread: ignoring TX echo\n");
            bus->expecting_tx_echo = false;
            continue;
        }

        // If we expected an echo but got something different, that's a collision
        if (bus->expecting_tx_echo) {
            BUS_DEBUG("RX thread: collision detected!\n");
            bus->expecting_tx_echo = false;
            // TODO: Handle collision
        }

        // Decode packet
        tsumikoro_packet_t rx_packet;
        size_t bytes_consumed = 0;
        tsumikoro_status_t status = tsumikoro_packet_decode(rx_msg.data,
                                                             rx_msg.len,
                                                             &rx_packet,
                                                             &bytes_consumed);

        if (status != TSUMIKORO_STATUS_OK) {
            BUS_DEBUG("RX thread: decode failed, status=%d\n", status);
            if (status == TSUMIKORO_STATUS_CRC_ERROR) {
                bus->stats.crc_errors++;
            }
            continue;
        }

        BUS_DEBUG("RX thread: decoded packet, cmd=0x%04X\n", rx_packet.command);

        // Post to handler thread
        tsumikoro_handler_event_t event;
        event.type = TSUMIKORO_HANDLER_EVENT_RX_PACKET;
        event.packet = rx_packet;
        tsumikoro_queue_send(bus->handler_queue, &event, UINT32_MAX);
    }
}

/**
 * @brief TX thread function
 *
 * Waits for TX requests from handler thread and transmits data via HAL.
 */
static void bus_tx_thread(void *arg)
{
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)arg;

    BUS_DEBUG("TX thread started\n");

    while (1) {
        tsumikoro_tx_msg_t tx_msg;

        // Block waiting for TX request
        if (!tsumikoro_queue_receive(bus->tx_queue, &tx_msg, UINT32_MAX)) {
            continue;
        }

        BUS_DEBUG("TX thread: transmitting %u bytes\n", (unsigned int)tx_msg.len);

        // Wait for bus idle (with timeout)
        uint32_t start_time = tsumikoro_hal_get_time_ms();
        while (!tsumikoro_hal_is_bus_idle(bus->hal)) {
            if (tsumikoro_hal_get_time_ms() - start_time >= bus->config.bus_idle_timeout_ms) {
                BUS_DEBUG("TX thread: bus idle timeout\n");
                // Notify handler of timeout
                tsumikoro_handler_event_t event;
                event.type = TSUMIKORO_HANDLER_EVENT_TIMEOUT;
                tsumikoro_queue_send(bus->handler_queue, &event, 0);
                goto skip_tx;
            }
            tsumikoro_thread_sleep_ms(1);
        }

        // Transmit
        tsumikoro_hal_status_t hal_status = tsumikoro_hal_transmit(bus->hal, tx_msg.data, tx_msg.len);

        if (hal_status == TSUMIKORO_HAL_OK) {
            // Save last TX for echo detection
            memcpy(bus->last_tx_buffer, tx_msg.data, tx_msg.len);
            bus->last_tx_len = tx_msg.len;
            bus->expecting_tx_echo = true;

            BUS_DEBUG("TX thread: transmission complete\n");

            // Notify handler of TX completion
            tsumikoro_handler_event_t event;
            event.type = TSUMIKORO_HANDLER_EVENT_TX_COMPLETE;
            tsumikoro_queue_send(bus->handler_queue, &event, 0);
        } else {
            BUS_DEBUG("TX thread: HAL transmit failed, status=%d\n", hal_status);
        }

skip_tx:
        (void)0;  // Label target
    }
}

/**
 * @brief Handler thread function
 *
 * Processes decoded RX packets, manages state machine, executes callbacks.
 */
static void bus_handler_thread(void *arg)
{
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)arg;

    BUS_DEBUG("Handler thread started\n");

    while (1) {
        tsumikoro_handler_event_t event;

        // Block waiting for events
        if (!tsumikoro_queue_receive(bus->handler_queue, &event, UINT32_MAX)) {
            continue;
        }

        switch (event.type) {
            case TSUMIKORO_HANDLER_EVENT_RX_PACKET:
                BUS_DEBUG("Handler: RX packet event\n");

                // Check if this is a response to pending command
                if (bus->pending_cmd.active && bus->pending_cmd.expects_response) {
                    if (bus->state == TSUMIKORO_BUS_STATE_WAITING_RESPONSE) {
                        // Check for NAK
                        if (event.packet.data_len > 0 && event.packet.data[0] == TSUMIKORO_STATUS_NAK) {
                            if (bus->config.auto_retry) {
                                bus_retry_command(bus);
                            } else {
                                bus_complete_command(bus, TSUMIKORO_CMD_STATUS_NAK, &event.packet);
                            }
                        } else {
                            // Success
                            bus_complete_command(bus, TSUMIKORO_CMD_STATUS_SUCCESS, &event.packet);
                        }
                        break;
                    }
                }

                // Unsolicited message - call callback
                if (bus->unsolicited_callback) {
                    BUS_DEBUG("Handler: calling unsolicited callback\n");
                    bus->unsolicited_callback(&event.packet, bus->unsolicited_user_data);
                }
                bus->stats.unsolicited_messages++;
                break;

            case TSUMIKORO_HANDLER_EVENT_TX_COMPLETE:
                BUS_DEBUG("Handler: TX complete event\n");
                if (bus->pending_cmd.expects_response) {
                    bus_set_state(bus, TSUMIKORO_BUS_STATE_WAITING_RESPONSE);
                } else {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_SUCCESS, NULL);
                }
                break;

            case TSUMIKORO_HANDLER_EVENT_TIMEOUT:
                BUS_DEBUG("Handler: timeout event\n");
                if (bus->config.auto_retry) {
                    bus_retry_command(bus);
                } else {
                    bus_complete_command(bus, TSUMIKORO_CMD_STATUS_TIMEOUT, NULL);
                }
                break;
        }

        // TODO: Add timeout checking for response timeout
    }
}

/**
 * @brief Initialize RTOS resources
 */
static bool bus_rtos_init(tsumikoro_bus_t *bus)
{
    // Create queues
    bus->rx_queue = tsumikoro_queue_create(8, sizeof(tsumikoro_rx_msg_t));
    if (!bus->rx_queue) goto error;

    bus->tx_queue = tsumikoro_queue_create(4, sizeof(tsumikoro_tx_msg_t));
    if (!bus->tx_queue) goto error;

    bus->handler_queue = tsumikoro_queue_create(8, sizeof(tsumikoro_handler_event_t));
    if (!bus->handler_queue) goto error;

    // Create semaphore
    bus->blocking_sem = tsumikoro_semaphore_create_binary();
    if (!bus->blocking_sem) goto error;

    // Create threads
    tsumikoro_thread_config_t thread_config;

    // RX thread (high priority)
    thread_config.name = "bus_rx";
    thread_config.function = bus_rx_thread;
    thread_config.argument = bus;
    thread_config.stack_size = 2048;
    thread_config.priority = TSUMIKORO_THREAD_PRIORITY_HIGH;
    bus->rx_thread = tsumikoro_thread_create(&thread_config);
    if (!bus->rx_thread) goto error;

    // TX thread (high priority)
    thread_config.name = "bus_tx";
    thread_config.function = bus_tx_thread;
    thread_config.argument = bus;
    thread_config.stack_size = 2048;
    thread_config.priority = TSUMIKORO_THREAD_PRIORITY_HIGH;
    bus->tx_thread = tsumikoro_thread_create(&thread_config);
    if (!bus->tx_thread) goto error;

    // Handler thread (normal priority)
    thread_config.name = "bus_handler";
    thread_config.function = bus_handler_thread;
    thread_config.argument = bus;
    thread_config.stack_size = 3072;
    thread_config.priority = TSUMIKORO_THREAD_PRIORITY_NORMAL;
    bus->handler_thread = tsumikoro_thread_create(&thread_config);
    if (!bus->handler_thread) goto error;

    BUS_DEBUG("RTOS resources initialized\n");
    return true;

error:
    // Cleanup on error
    if (bus->rx_queue) tsumikoro_queue_delete(bus->rx_queue);
    if (bus->tx_queue) tsumikoro_queue_delete(bus->tx_queue);
    if (bus->handler_queue) tsumikoro_queue_delete(bus->handler_queue);
    if (bus->blocking_sem) tsumikoro_semaphore_delete(bus->blocking_sem);
    return false;
}

/**
 * @brief Cleanup RTOS resources
 */
static void bus_rtos_deinit(tsumikoro_bus_t *bus)
{
    // Delete threads
    if (bus->rx_thread) tsumikoro_thread_delete(bus->rx_thread);
    if (bus->tx_thread) tsumikoro_thread_delete(bus->tx_thread);
    if (bus->handler_thread) tsumikoro_thread_delete(bus->handler_thread);

    // Delete queues
    if (bus->rx_queue) tsumikoro_queue_delete(bus->rx_queue);
    if (bus->tx_queue) tsumikoro_queue_delete(bus->tx_queue);
    if (bus->handler_queue) tsumikoro_queue_delete(bus->handler_queue);

    // Delete semaphore
    if (bus->blocking_sem) tsumikoro_semaphore_delete(bus->blocking_sem);

    BUS_DEBUG("RTOS resources cleaned up\n");
}

#endif /* TSUMIKORO_BUS_USE_RTOS */

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

#if TSUMIKORO_BUS_USE_RTOS
    // Initialize RTOS resources
    if (!bus_rtos_init(bus)) {
        free(bus);
        return NULL;
    }

    // Note: In RTOS mode, the HAL must be re-initialized with RX callback.
    // The application should call tsumikoro_hal_init() again with
    // bus_hal_rx_callback as the RX callback and bus as user_data.
    // This can't be done here because HAL init requires platform config.
    BUS_DEBUG("Bus initialized in RTOS mode\n");
#else
    BUS_DEBUG("Bus initialized in bare-metal mode\n");
#endif

    return (tsumikoro_bus_handle_t)bus;
}

void tsumikoro_bus_deinit(tsumikoro_bus_handle_t handle)
{
    if (handle) {
#if TSUMIKORO_BUS_USE_RTOS
        tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
        bus_rtos_deinit(bus);
#endif
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

    BUS_DEBUG("send_command_blocking: device=0x%02X, cmd=0x%04X, timeout=%ums\n",
              packet->device_id, packet->command, timeout_ms);

    // Use bus default timeout if not specified
    if (timeout_ms == 0) {
        timeout_ms = bus->config.response_timeout_ms;
    }

    // Send command asynchronously using context struct for C11 compatibility
    volatile tsumikoro_cmd_status_t result = TSUMIKORO_CMD_STATUS_PENDING;
    volatile tsumikoro_packet_t temp_response;
    volatile bool done = false;

    // Setup blocking callback context
    tsumikoro_blocking_callback_ctx_t cb_ctx = {
        .result = &result,
        .temp_response = &temp_response,
        .done = &done,
        .response = response
    };

    tsumikoro_status_t send_status = tsumikoro_bus_send_command_async(handle, packet,
                                                                       bus_blocking_callback, &cb_ctx);
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

#if TSUMIKORO_BUS_USE_RTOS
    // In RTOS mode, threads handle all processing - no need to call this
    (void)handle;
#else
    // Bare-metal mode: process state machine
    tsumikoro_bus_t *bus = (tsumikoro_bus_t *)handle;
    bus->last_process_time_ms = tsumikoro_hal_get_time_ms();

    bus_process_state_machine(bus);
#endif
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

#if TSUMIKORO_BUS_USE_RTOS
tsumikoro_hal_rx_callback_t tsumikoro_bus_get_hal_rx_callback(void)
{
    return bus_hal_rx_callback;
}
#endif
