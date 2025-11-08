/**
 * @file tsumikoro_hal_esp32.c
 * @brief ESP32 HAL implementation for Tsumikoro bus
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_hal_esp32.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tsumikoro_hal_esp32";

/* Debug logging */
#ifdef TSUMIKORO_HAL_DEBUG
    #define HAL_DEBUG(fmt, ...) ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
    #define HAL_DEBUG_BUFFER(data, len) ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_DEBUG)
#else
    #define HAL_DEBUG(...) ((void)0)
    #define HAL_DEBUG_BUFFER(data, len) ((void)0)
#endif

/**
 * @brief Maximum receive buffer size
 */
#define TSUMIKORO_ESP32_RX_BUFFER_SIZE 256

/**
 * @brief ESP32 device context
 */
typedef struct {
    // Configuration
    const tsumikoro_hal_esp32_config_t *esp32_config;
    uint8_t device_id;
    bool is_controller;
    uint32_t baud_rate;
    uint32_t turnaround_delay_bytes;

    // Callback
    tsumikoro_hal_rx_callback_t rx_callback;
    void *user_data;

    // RX buffering
    uint8_t rx_buffer[TSUMIKORO_ESP32_RX_BUFFER_SIZE];
    volatile size_t rx_head;
    volatile size_t rx_tail;
    volatile size_t rx_count;

    // State
    volatile bool tx_active;
    volatile uint32_t last_activity_tick;
} tsumikoro_esp32_device_t;

bool tsumikoro_hal_esp32_init_peripheral(const tsumikoro_hal_esp32_config_t *config,
                                          uint32_t baud_rate)
{
    if (!config) {
        return false;
    }

    // UART configuration
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver
    esp_err_t err = uart_driver_install(config->uart_port,
                                         config->rx_buffer_size,
                                         config->tx_buffer_size,
                                         0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return false;
    }

    // Configure UART parameters
    err = uart_param_config(config->uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
        return false;
    }

    // Set UART pins
    err = uart_set_pin(config->uart_port,
                       config->tx_pin,
                       config->rx_pin,
                       config->rts_pin,
                       config->cts_pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        return false;
    }

    // Configure RS-485 mode if enabled
    if (config->use_rs485_mode && config->rts_pin >= 0) {
        uart_set_mode(config->uart_port, UART_MODE_RS485_HALF_DUPLEX);
        ESP_LOGI(TAG, "RS-485 half-duplex mode enabled");
    }

    ESP_LOGI(TAG, "UART%d initialized: %d baud, TX=%d, RX=%d, RTS/DE=%d",
             config->uart_port, baud_rate,
             config->tx_pin, config->rx_pin, config->rts_pin);

    return true;
}

tsumikoro_hal_handle_t tsumikoro_hal_init(const tsumikoro_hal_config_t *config,
                                           tsumikoro_hal_rx_callback_t rx_callback,
                                           void *user_data)
{
    if (!config || !config->platform_data) {
        ESP_LOGE(TAG, "Invalid configuration");
        return NULL;
    }

    const tsumikoro_hal_esp32_config_t *esp32_config =
        (const tsumikoro_hal_esp32_config_t *)config->platform_data;

    // Allocate device context
    tsumikoro_esp32_device_t *device = calloc(1, sizeof(tsumikoro_esp32_device_t));
    if (!device) {
        ESP_LOGE(TAG, "Failed to allocate device context");
        return NULL;
    }

    // Initialize device context
    device->esp32_config = esp32_config;
    device->device_id = config->device_id;
    device->is_controller = config->is_controller;
    device->baud_rate = config->baud_rate;
    device->turnaround_delay_bytes = config->turnaround_delay_bytes;
    device->rx_callback = rx_callback;

    ESP_LOGI(TAG, "HAL init: device_id=0x%02X, controller=%d, baud=%u",
             device->device_id, device->is_controller, device->baud_rate);
    device->user_data = user_data;
    device->rx_head = 0;
    device->rx_tail = 0;
    device->rx_count = 0;
    device->tx_active = false;
    device->last_activity_tick = 0;

    HAL_DEBUG("HAL initialized: Device ID=0x%02X, %s, RX callback=%p",
              device->device_id,
              device->is_controller ? "Controller" : "Peripheral",
              (void*)device->rx_callback);

    return (tsumikoro_hal_handle_t)device;
}

void tsumikoro_hal_deinit(tsumikoro_hal_handle_t handle)
{
    if (!handle) {
        return;
    }

    tsumikoro_esp32_device_t *device = (tsumikoro_esp32_device_t *)handle;

    // Delete UART driver
    uart_driver_delete(device->esp32_config->uart_port);

    free(device);
}

tsumikoro_hal_status_t tsumikoro_hal_transmit(tsumikoro_hal_handle_t handle,
                                               const uint8_t *data,
                                               size_t len)
{
    if (!handle || !data || len == 0) {
        ESP_LOGE(TAG, "TX: Invalid parameters (handle=%p, data=%p, len=%zu)", handle, data, len);
        return TSUMIKORO_HAL_ERROR;
    }

    tsumikoro_esp32_device_t *device = (tsumikoro_esp32_device_t *)handle;

    if (device->tx_active) {
        ESP_LOGW(TAG, "TX: Already in progress");
        return TSUMIKORO_HAL_BUSY;
    }

    HAL_DEBUG("TX: Sending %zu bytes", len);
    HAL_DEBUG_BUFFER(data, len);

    device->tx_active = true;
    device->last_activity_tick = xTaskGetTickCount();

#if TSUMIKORO_IGNORE_RX_DURING_TX
    // Flush RX buffer before transmitting to avoid processing echo
    size_t flushed = 0;
    uart_get_buffered_data_len(device->esp32_config->uart_port, &flushed);
    if (flushed > 0) {
        ESP_LOGD(TAG, "TX: Flushing %zu bytes from RX buffer before TX", flushed);
    }
    uart_flush_input(device->esp32_config->uart_port);
#endif

    // Transmit data using UART
    int written = uart_write_bytes(device->esp32_config->uart_port, data, len);
    if (written != (int)len) {
        ESP_LOGE(TAG, "TX: Write failed - wrote %d/%zu bytes", written, len);
    }

    // Wait for transmission to complete
    esp_err_t wait_result = uart_wait_tx_done(device->esp32_config->uart_port, pdMS_TO_TICKS(100));
    if (wait_result != ESP_OK) {
        ESP_LOGE(TAG, "TX: Wait for TX done failed: %s", esp_err_to_name(wait_result));
    } else {
        ESP_LOGD(TAG, "TX: Complete");
    }

#if TSUMIKORO_IGNORE_RX_DURING_TX
    // Flush RX buffer after transmitting to discard echo data
    flushed = 0;
    uart_get_buffered_data_len(device->esp32_config->uart_port, &flushed);
    if (flushed > 0) {
        ESP_LOGD(TAG, "TX: Flushing %zu bytes from RX buffer after TX (echo)", flushed);
    }
    uart_flush_input(device->esp32_config->uart_port);
#endif

    device->tx_active = false;

    return (written == (int)len) ? TSUMIKORO_HAL_OK : TSUMIKORO_HAL_ERROR;
}

tsumikoro_hal_status_t tsumikoro_hal_receive(tsumikoro_hal_handle_t handle,
                                              uint8_t *buffer,
                                              size_t max_len,
                                              size_t *received_len,
                                              uint32_t timeout_ms)
{
    if (!handle || !buffer || max_len == 0 || !received_len) {
        return TSUMIKORO_HAL_ERROR;
    }

    tsumikoro_esp32_device_t *device = (tsumikoro_esp32_device_t *)handle;
    *received_len = 0;

    // Check if RX callback is registered (RTOS mode)
    if (device->rx_callback) {
        // In RTOS mode with callback, read directly from UART and invoke callback
        int len = uart_read_bytes(device->esp32_config->uart_port,
                                   buffer, max_len,
                                   pdMS_TO_TICKS(timeout_ms));

        if (len > 0) {
            HAL_DEBUG("RX: Received %d bytes", len);
            HAL_DEBUG_BUFFER(buffer, len);

            device->last_activity_tick = xTaskGetTickCount();
            *received_len = len;
            device->rx_callback(buffer, len, device->user_data);
            return TSUMIKORO_HAL_OK;
        } else if (len == 0) {
            HAL_DEBUG("RX: Timeout (no data received)");
        } else {
            ESP_LOGE(TAG, "RX: Error reading UART (%d)", len);
        }

        return (len == 0) ? TSUMIKORO_HAL_TIMEOUT : TSUMIKORO_HAL_ERROR;
    }

    // Bare-metal mode: use internal circular buffer
    size_t count = 0;

    // First, drain circular buffer
    while (count < max_len && device->rx_count > 0) {
        buffer[count++] = device->rx_buffer[device->rx_tail];
        device->rx_tail = (device->rx_tail + 1) % TSUMIKORO_ESP32_RX_BUFFER_SIZE;
        device->rx_count--;
    }

    // Try to read more data from UART
    if (count < max_len) {
        uint8_t temp_buf[64];
        int len = uart_read_bytes(device->esp32_config->uart_port,
                                   temp_buf, sizeof(temp_buf),
                                   pdMS_TO_TICKS(timeout_ms));

        if (len > 0) {
            device->last_activity_tick = xTaskGetTickCount();

            // Copy to output buffer
            size_t to_copy = (len < (int)(max_len - count)) ? len : (max_len - count);
            memcpy(&buffer[count], temp_buf, to_copy);
            count += to_copy;

            // Store remaining data in circular buffer
            for (int i = to_copy; i < len; i++) {
                if (device->rx_count < TSUMIKORO_ESP32_RX_BUFFER_SIZE) {
                    device->rx_buffer[device->rx_head] = temp_buf[i];
                    device->rx_head = (device->rx_head + 1) % TSUMIKORO_ESP32_RX_BUFFER_SIZE;
                    device->rx_count++;
                }
            }
        }
    }

    *received_len = count;
    return (count > 0) ? TSUMIKORO_HAL_OK : TSUMIKORO_HAL_TIMEOUT;
}

bool tsumikoro_hal_is_bus_idle(tsumikoro_hal_handle_t handle)
{
    if (!handle) {
        return true;
    }

    tsumikoro_esp32_device_t *device = (tsumikoro_esp32_device_t *)handle;

    // Check if TX is active
    if (device->tx_active) {
        return false;
    }

    // Check turnaround delay
    if (device->turnaround_delay_bytes > 0) {
        // Calculate required delay in ms
        uint32_t delay_us = (10 * 1000000ULL / device->baud_rate) * device->turnaround_delay_bytes;
        uint32_t delay_ms = (delay_us + 999) / 1000;  // Round up
        uint32_t delay_ticks = pdMS_TO_TICKS(delay_ms);

        uint32_t now = xTaskGetTickCount();
        uint32_t elapsed = now - device->last_activity_tick;

        if (elapsed < delay_ticks) {
            return false;  // Still in turnaround delay
        }
    }

    return true;
}

void tsumikoro_hal_enable_tx(tsumikoro_hal_handle_t handle)
{
    // ESP32 RS-485 mode handles this automatically with RTS pin
    // No action needed
    (void)handle;
}

void tsumikoro_hal_disable_tx(tsumikoro_hal_handle_t handle)
{
    // ESP32 RS-485 mode handles this automatically with RTS pin
    // No action needed
    (void)handle;
}

void tsumikoro_hal_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

uint32_t tsumikoro_hal_get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}
