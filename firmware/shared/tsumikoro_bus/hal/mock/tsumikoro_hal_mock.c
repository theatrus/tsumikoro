/**
 * @file tsumikoro_hal_mock.c
 * @brief Mock HAL implementation for testing
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#define _POSIX_C_SOURCE 199309L

#include "tsumikoro_hal_mock.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* Debug logging (define TSUMIKORO_HAL_MOCK_DEBUG to enable) */
#ifdef TSUMIKORO_HAL_MOCK_DEBUG
#define HAL_DEBUG(...) printf("[HAL-MOCK] " __VA_ARGS__)
#else
#define HAL_DEBUG(...) ((void)0)
#endif

/**
 * @brief Mock device structure
 */
typedef struct {
    uint8_t device_id;
    bool is_controller;
    bool tx_enabled;
    tsumikoro_hal_rx_callback_t rx_callback;
    void *user_data;

    // Reference to bus
    struct tsumikoro_mock_bus *bus;

    // RX buffer for polling mode
    uint8_t rx_buffer[TSUMIKORO_MOCK_BUS_BUFFER_SIZE];
    size_t rx_buffer_head;
    size_t rx_buffer_tail;
    size_t rx_buffer_count;
} tsumikoro_mock_device_t;

/**
 * @brief Mock bus structure (shared between devices)
 */
typedef struct tsumikoro_mock_bus {
    // Connected devices
    tsumikoro_mock_device_t *devices[TSUMIKORO_MOCK_MAX_DEVICES];
    size_t device_count;

    // Shared bus buffer (simulates wire)
    uint8_t bus_buffer[TSUMIKORO_MOCK_BUS_BUFFER_SIZE];
    size_t bus_buffer_len;
    bool bus_active;
    tsumikoro_mock_device_t *transmitting_device;  // Track who transmitted current packet

    // Time simulation
    uint32_t current_time_ms;

    // Statistics
    tsumikoro_mock_bus_stats_t stats;

    // Error injection
    tsumikoro_mock_error_type_t pending_error;
} tsumikoro_mock_bus_t;

/* ========== Global Simulated Time ========== */

// Global simulated time (used by tsumikoro_hal_get_time_ms())
// Controlled by tsumikoro_mock_bus_process() and tsumikoro_hal_delay_us()
static uint32_t g_simulated_time_ms = 0;

// Global list of active buses (so tsumikoro_hal_delay_us() can process them)
static tsumikoro_mock_bus_t *g_active_buses[TSUMIKORO_MOCK_MAX_DEVICES] = {0};
static size_t g_active_bus_count = 0;

/* ========== Bus Management ========== */

tsumikoro_mock_bus_handle_t tsumikoro_mock_bus_create(void)
{
    tsumikoro_mock_bus_t *bus = (tsumikoro_mock_bus_t *)calloc(1, sizeof(tsumikoro_mock_bus_t));
    if (bus == NULL) {
        return NULL;
    }

    // Add to global list
    if (g_active_bus_count < TSUMIKORO_MOCK_MAX_DEVICES) {
        g_active_buses[g_active_bus_count++] = bus;
    }

    return bus;
}

void tsumikoro_mock_bus_destroy(tsumikoro_mock_bus_handle_t bus)
{
    if (bus) {
        // Remove from global list
        for (size_t i = 0; i < g_active_bus_count; i++) {
            if (g_active_buses[i] == bus) {
                // Shift remaining buses down
                for (size_t j = i; j < g_active_bus_count - 1; j++) {
                    g_active_buses[j] = g_active_buses[j + 1];
                }
                g_active_buses[--g_active_bus_count] = NULL;
                break;
            }
        }
        free(bus);
    }
}

/* ========== HAL Interface Implementation ========== */

tsumikoro_hal_handle_t tsumikoro_hal_init(const tsumikoro_hal_config_t *config,
                                           tsumikoro_hal_rx_callback_t rx_callback,
                                           void *user_data)
{
    if (config == NULL || config->platform_data == NULL) {
        return NULL;
    }

    // Platform_data contains the bus handle for mock HAL
    tsumikoro_mock_bus_t *bus = (tsumikoro_mock_bus_t *)config->platform_data;

    // Create device
    tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)calloc(1, sizeof(tsumikoro_mock_device_t));
    if (device == NULL) {
        return NULL;
    }

    device->device_id = config->device_id;
    device->is_controller = config->is_controller;
    device->rx_callback = rx_callback;
    device->user_data = user_data;
    device->tx_enabled = false;
    device->bus = bus;  // Store bus reference

    // Add to bus
    if (bus->device_count >= TSUMIKORO_MOCK_MAX_DEVICES) {
        free(device);
        return NULL;
    }

    bus->devices[bus->device_count++] = device;

    return (tsumikoro_hal_handle_t)device;
}

tsumikoro_hal_handle_t tsumikoro_mock_create_device(tsumikoro_mock_bus_handle_t bus,
                                                     const tsumikoro_hal_config_t *config,
                                                     tsumikoro_hal_rx_callback_t rx_callback,
                                                     void *user_data)
{
    if (bus == NULL || config == NULL) {
        return NULL;
    }

    // Copy config and set platform_data to bus handle
    tsumikoro_hal_config_t mock_config = *config;
    mock_config.platform_data = (void *)bus;

    return tsumikoro_hal_init(&mock_config, rx_callback, user_data);
}

void tsumikoro_hal_deinit(tsumikoro_hal_handle_t handle)
{
    if (handle) {
        tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)handle;
        free(device);
    }
}

tsumikoro_hal_status_t tsumikoro_hal_transmit(tsumikoro_hal_handle_t handle,
                                               const uint8_t *data,
                                               size_t len)
{
    if (handle == NULL || data == NULL || len == 0) {
        return TSUMIKORO_HAL_ERROR;
    }

    tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)handle;
    tsumikoro_mock_bus_t *bus = device->bus;

    if (bus == NULL) {
        return TSUMIKORO_HAL_ERROR;
    }

    // Check if bus is busy
    if (bus->bus_active || bus->bus_buffer_len > 0) {
        return TSUMIKORO_HAL_BUSY;
    }

    // Check buffer size
    if (len > TSUMIKORO_MOCK_BUS_BUFFER_SIZE) {
        return TSUMIKORO_HAL_ERROR;
    }

    // Copy data to bus buffer
    memcpy(bus->bus_buffer, data, len);
    bus->bus_buffer_len = len;
    bus->bus_active = true;
    bus->transmitting_device = device;  // Track transmitting device

    // Update stats
    bus->stats.packets_transmitted++;

    // Apply error injection if configured
    if (bus->pending_error != TSUMIKORO_MOCK_ERROR_NONE) {
        switch (bus->pending_error) {
            case TSUMIKORO_MOCK_ERROR_CRC_CORRUPT:
                if (len >= 2) {
                    // Corrupt CRC byte (second-to-last byte in our protocol)
                    bus->bus_buffer[len - 2] ^= 0xFF;
                    bus->stats.crc_errors_injected++;
                }
                break;

            case TSUMIKORO_MOCK_ERROR_DROP_PACKET:
                bus->bus_buffer_len = 0;
                bus->bus_active = false;
                bus->stats.packets_dropped++;
                bus->pending_error = TSUMIKORO_MOCK_ERROR_NONE;
                return TSUMIKORO_HAL_OK;

            case TSUMIKORO_MOCK_ERROR_TRUNCATE:
                if (len > 3) {
                    bus->bus_buffer_len = len / 2;
                }
                break;

            case TSUMIKORO_MOCK_ERROR_BIT_FLIP:
                if (len > 0) {
                    size_t byte_idx = rand() % len;
                    uint8_t bit_idx = rand() % 8;
                    bus->bus_buffer[byte_idx] ^= (1 << bit_idx);
                }
                break;

            default:
                break;
        }

        bus->pending_error = TSUMIKORO_MOCK_ERROR_NONE;
    }

    return TSUMIKORO_HAL_OK;
}

tsumikoro_hal_status_t tsumikoro_hal_receive(tsumikoro_hal_handle_t handle,
                                              uint8_t *buffer,
                                              size_t max_len,
                                              size_t *received_len,
                                              uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL || received_len == NULL) {
        return TSUMIKORO_HAL_ERROR;
    }

    tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)handle;

    // Check if data available in RX buffer
    if (device->rx_buffer_count == 0) {
        *received_len = 0;
        return (timeout_ms == 0) ? TSUMIKORO_HAL_NOT_READY : TSUMIKORO_HAL_TIMEOUT;
    }

    // Copy from circular buffer
    size_t to_copy = (device->rx_buffer_count < max_len) ? device->rx_buffer_count : max_len;
    for (size_t i = 0; i < to_copy; i++) {
        buffer[i] = device->rx_buffer[device->rx_buffer_tail];
        device->rx_buffer_tail = (device->rx_buffer_tail + 1) % TSUMIKORO_MOCK_BUS_BUFFER_SIZE;
        device->rx_buffer_count--;
    }

    *received_len = to_copy;

    #ifdef TSUMIKORO_HAL_MOCK_DEBUG
    if (to_copy > 0 && to_copy < 20) {
        HAL_DEBUG("Device 0x%02X received %zu bytes:", device->device_id, to_copy);
        for (size_t i = 0; i < to_copy && i < 15; i++) {
            printf(" %02X", buffer[i]);
        }
        printf("\n");
    }
    #endif

    return TSUMIKORO_HAL_OK;
}

bool tsumikoro_hal_is_bus_idle(tsumikoro_hal_handle_t handle)
{
    (void)handle;
    // In mock implementation, always return true (no real contention)
    return true;
}

void tsumikoro_hal_enable_tx(tsumikoro_hal_handle_t handle)
{
    if (handle) {
        tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)handle;
        device->tx_enabled = true;
    }
}

void tsumikoro_hal_disable_tx(tsumikoro_hal_handle_t handle)
{
    if (handle) {
        tsumikoro_mock_device_t *device = (tsumikoro_mock_device_t *)handle;
        device->tx_enabled = false;
    }
}

void tsumikoro_hal_delay_us(uint32_t us)
{
    // Advance simulated time when delay is called
    // This allows blocking operations to advance time naturally
    uint32_t ms = (us + 999) / 1000;  // Round up to milliseconds
    if (ms > 0) {
        g_simulated_time_ms += ms;

        // Process all active buses to simulate packet delivery during delay
        // This is critical for blocking send operations to work correctly
        for (size_t i = 0; i < g_active_bus_count; i++) {
            tsumikoro_mock_bus_process(g_active_buses[i], 0);  // Don't advance time again
        }
    }
}

/* ========== Mock Bus Processing ========== */

uint32_t tsumikoro_hal_get_time_ms(void)
{
    // Return simulated time controlled by tsumikoro_mock_bus_process()
    // and tsumikoro_hal_delay_us()
    return g_simulated_time_ms;
}

void tsumikoro_mock_bus_process(tsumikoro_mock_bus_handle_t bus, uint32_t time_advance_ms)
{
    if (bus == NULL) {
        return;
    }

    // Advance global simulated time
    g_simulated_time_ms += time_advance_ms;
    bus->current_time_ms = g_simulated_time_ms;

    // Process any pending transmissions
    if (bus->bus_buffer_len > 0) {
        HAL_DEBUG("Processing %zu bytes on bus, transmitter=0x%02X\n",
                  bus->bus_buffer_len,
                  bus->transmitting_device ? bus->transmitting_device->device_id : 0xFF);

        // Copy packet data before clearing (callbacks may need to access it)
        uint8_t packet_buffer[TSUMIKORO_MOCK_BUS_BUFFER_SIZE];
        size_t packet_len = bus->bus_buffer_len;
        memcpy(packet_buffer, bus->bus_buffer, packet_len);

        // Clear bus BEFORE invoking callbacks so peripherals can respond immediately
        bus->bus_buffer_len = 0;
        bus->bus_active = false;
        bus->transmitting_device = NULL;
        bus->stats.packets_received += bus->device_count;

        // Broadcast to ALL devices (including transmitter for collision detection)
        // In RS-485, if RE is asserted during transmission, the device receives its own packets
        // This allows for collision detection by comparing TX vs RX
        for (size_t i = 0; i < bus->device_count; i++) {
            tsumikoro_mock_device_t *device = bus->devices[i];

            HAL_DEBUG("Delivering to device 0x%02X (callback=%s)\n",
                      device->device_id, device->rx_callback ? "yes" : "no");

            if (device->rx_callback) {
                // Deliver via callback
                device->rx_callback(packet_buffer, packet_len, device->user_data);
            } else {
                // Store in RX buffer for polling
                for (size_t j = 0; j < packet_len && device->rx_buffer_count < TSUMIKORO_MOCK_BUS_BUFFER_SIZE; j++) {
                    device->rx_buffer[device->rx_buffer_head] = packet_buffer[j];
                    device->rx_buffer_head = (device->rx_buffer_head + 1) % TSUMIKORO_MOCK_BUS_BUFFER_SIZE;
                    device->rx_buffer_count++;
                }
            }
        }
    }
}

size_t tsumikoro_mock_bus_get_pending_bytes(tsumikoro_mock_bus_handle_t bus)
{
    return bus ? bus->bus_buffer_len : 0;
}

void tsumikoro_mock_bus_inject_error(tsumikoro_mock_bus_handle_t bus,
                                      tsumikoro_mock_error_type_t error_type)
{
    if (bus) {
        bus->pending_error = error_type;
    }
}

void tsumikoro_mock_bus_get_stats(tsumikoro_mock_bus_handle_t bus,
                                   tsumikoro_mock_bus_stats_t *stats)
{
    if (bus && stats) {
        *stats = bus->stats;
    }
}

void tsumikoro_mock_bus_reset_stats(tsumikoro_mock_bus_handle_t bus)
{
    if (bus) {
        memset(&bus->stats, 0, sizeof(bus->stats));
    }
}
