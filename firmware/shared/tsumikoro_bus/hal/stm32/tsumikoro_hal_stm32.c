/**
 * @file tsumikoro_hal_stm32.c
 * @brief STM32 HAL implementation for Tsumikoro bus
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_hal_stm32.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief Maximum receive buffer size
 */
#define TSUMIKORO_STM32_RX_BUFFER_SIZE 256

/**
 * @brief STM32 device context
 */
typedef struct {
    // Configuration
    UART_HandleTypeDef uart_handle;
    const tsumikoro_hal_stm32_config_t *stm32_config;
    uint8_t device_id;
    bool is_controller;
    uint32_t baud_rate;
    uint32_t turnaround_delay_bytes;

    // Callback
    tsumikoro_hal_rx_callback_t rx_callback;
    void *user_data;

    // RX buffering
    volatile uint8_t rx_buffer[TSUMIKORO_STM32_RX_BUFFER_SIZE];
    volatile size_t rx_head;
    volatile size_t rx_tail;
    volatile size_t rx_count;

    // DMA handles (if used)
    DMA_HandleTypeDef dma_tx_handle;
    DMA_HandleTypeDef dma_rx_handle;
    bool use_dma;

    // State
    volatile bool tx_active;
    volatile uint32_t last_activity_tick;
} tsumikoro_stm32_device_t;

/* ========== STM32 Peripheral Initialization ========== */

bool tsumikoro_hal_stm32_init_peripheral(const tsumikoro_hal_stm32_config_t *config,
                                          uint32_t baud_rate)
{
    if (config == NULL || config->uart_instance == NULL) {
        return false;
    }

    // Enable UART clock (example for USART1 on STM32G0)
    // Note: This needs to be adapted for different UART instances
    if (config->uart_instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
    } else if (config->uart_instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
    }
    // Add more UART instances as needed

    // Note: GPIO initialization should be done by the application
    // since pin assignments vary by board

    // Enable DE/RE pins (RS-485 transceiver control)
    if (config->de_port != NULL) {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin = config->de_pin;
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(config->de_port, &gpio_init);

        // Initially disable driver (DE = 0)
        HAL_GPIO_WritePin(config->de_port, config->de_pin, GPIO_PIN_RESET);
    }

    if (config->re_port != NULL) {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin = config->re_pin;
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(config->re_port, &gpio_init);

        // Enable receiver (RE = 0 for active-low RE)
        HAL_GPIO_WritePin(config->re_port, config->re_pin, GPIO_PIN_RESET);
    }

    return true;
}

/* ========== HAL Interface Implementation ========== */

tsumikoro_hal_handle_t tsumikoro_hal_init(const tsumikoro_hal_config_t *config,
                                           tsumikoro_hal_rx_callback_t rx_callback,
                                           void *user_data)
{
    if (config == NULL || config->platform_data == NULL) {
        return NULL;
    }

    const tsumikoro_hal_stm32_config_t *stm32_config =
        (const tsumikoro_hal_stm32_config_t *)config->platform_data;

    // Allocate device context
    tsumikoro_stm32_device_t *device =
        (tsumikoro_stm32_device_t *)malloc(sizeof(tsumikoro_stm32_device_t));
    if (device == NULL) {
        return NULL;
    }

    memset(device, 0, sizeof(tsumikoro_stm32_device_t));

    // Store configuration
    device->stm32_config = stm32_config;
    device->device_id = config->device_id;
    device->is_controller = config->is_controller;
    device->baud_rate = config->baud_rate;
    device->turnaround_delay_bytes = config->turnaround_delay_bytes;
    device->rx_callback = rx_callback;
    device->user_data = user_data;
    device->use_dma = (stm32_config->dma_tx != NULL && stm32_config->dma_rx != NULL);

    // Configure UART
    device->uart_handle.Instance = stm32_config->uart_instance;
    device->uart_handle.Init.BaudRate = config->baud_rate;
    device->uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    device->uart_handle.Init.StopBits = UART_STOPBITS_1;
    device->uart_handle.Init.Parity = UART_PARITY_NONE;
    device->uart_handle.Init.Mode = UART_MODE_TX_RX;
    device->uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    device->uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&device->uart_handle) != HAL_OK) {
        free(device);
        return NULL;
    }

    // Configure DMA if enabled
    if (device->use_dma) {
        // TX DMA
        device->dma_tx_handle.Instance = stm32_config->dma_tx;
        device->dma_tx_handle.Init.Request = stm32_config->dma_tx_request;
        device->dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
        device->dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
        device->dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
        device->dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        device->dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        device->dma_tx_handle.Init.Mode = DMA_NORMAL;
        device->dma_tx_handle.Init.Priority = DMA_PRIORITY_HIGH;

        if (HAL_DMA_Init(&device->dma_tx_handle) != HAL_OK) {
            free(device);
            return NULL;
        }

        __HAL_LINKDMA(&device->uart_handle, hdmatx, device->dma_tx_handle);

        // RX DMA
        device->dma_rx_handle.Instance = stm32_config->dma_rx;
        device->dma_rx_handle.Init.Request = stm32_config->dma_rx_request;
        device->dma_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
        device->dma_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
        device->dma_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
        device->dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        device->dma_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        device->dma_rx_handle.Init.Mode = DMA_CIRCULAR;
        device->dma_rx_handle.Init.Priority = DMA_PRIORITY_HIGH;

        if (HAL_DMA_Init(&device->dma_rx_handle) != HAL_OK) {
            free(device);
            return NULL;
        }

        __HAL_LINKDMA(&device->uart_handle, hdmarx, device->dma_rx_handle);

        // Start DMA RX in circular mode
        HAL_UART_Receive_DMA(&device->uart_handle,
                            (uint8_t *)device->rx_buffer,
                            TSUMIKORO_STM32_RX_BUFFER_SIZE);
    } else {
        // Enable UART RX interrupt for non-DMA mode
        __HAL_UART_ENABLE_IT(&device->uart_handle, UART_IT_RXNE);
    }

    // Enable UART interrupts
    HAL_NVIC_SetPriority(stm32_config->uart_irq, 0, 0);
    HAL_NVIC_EnableIRQ(stm32_config->uart_irq);

    if (device->use_dma) {
        HAL_NVIC_SetPriority(stm32_config->dma_tx_irq, 0, 0);
        HAL_NVIC_EnableIRQ(stm32_config->dma_tx_irq);

        HAL_NVIC_SetPriority(stm32_config->dma_rx_irq, 0, 0);
        HAL_NVIC_EnableIRQ(stm32_config->dma_rx_irq);
    }

    return (tsumikoro_hal_handle_t)device;
}

void tsumikoro_hal_deinit(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Disable interrupts
    HAL_NVIC_DisableIRQ(device->stm32_config->uart_irq);
    if (device->use_dma) {
        HAL_NVIC_DisableIRQ(device->stm32_config->dma_tx_irq);
        HAL_NVIC_DisableIRQ(device->stm32_config->dma_rx_irq);
    }

    // Deinitialize UART
    HAL_UART_DeInit(&device->uart_handle);

    // Deinitialize DMA
    if (device->use_dma) {
        HAL_DMA_DeInit(&device->dma_tx_handle);
        HAL_DMA_DeInit(&device->dma_rx_handle);
    }

    free(device);
}

tsumikoro_hal_status_t tsumikoro_hal_transmit(tsumikoro_hal_handle_t handle,
                                               const uint8_t *data,
                                               size_t len)
{
    if (handle == NULL || data == NULL || len == 0) {
        return TSUMIKORO_HAL_ERROR;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Check if TX is already in progress
    if (device->tx_active) {
        return TSUMIKORO_HAL_BUSY;
    }

    device->tx_active = true;
    device->last_activity_tick = HAL_GetTick();

#ifdef TSUMIKORO_HAL_MOCK_DEBUG
    // Debug: print transmitted bytes in hex
    printf("[HAL_TX] %u bytes:", (unsigned int)len);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
#endif

    // Enable RS-485 driver (DE high)
    if (device->stm32_config->de_port != NULL) {
        HAL_GPIO_WritePin(device->stm32_config->de_port,
                         device->stm32_config->de_pin,
                         GPIO_PIN_SET);
    }

    HAL_StatusTypeDef status;
    if (device->use_dma) {
        status = HAL_UART_Transmit_DMA(&device->uart_handle, (uint8_t *)data, len);
    } else {
        status = HAL_UART_Transmit_IT(&device->uart_handle, (uint8_t *)data, len);
    }

    if (status != HAL_OK) {
        device->tx_active = false;
        // Disable driver on error
        if (device->stm32_config->de_port != NULL) {
            HAL_GPIO_WritePin(device->stm32_config->de_port,
                             device->stm32_config->de_pin,
                             GPIO_PIN_RESET);
        }
#ifdef TSUMIKORO_HAL_MOCK_DEBUG
        printf("[HAL_TX] ERROR: HAL_UART_Transmit_DMA failed, status=%d, UART gState=%d\n",
               status, device->uart_handle.gState);
#endif
        return TSUMIKORO_HAL_ERROR;
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

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Check if data available in RX buffer
    if (device->rx_count == 0) {
        *received_len = 0;
        return (timeout_ms == 0) ? TSUMIKORO_HAL_NOT_READY : TSUMIKORO_HAL_TIMEOUT;
    }

    // Copy from circular buffer
    size_t to_copy = (device->rx_count < max_len) ? device->rx_count : max_len;
    for (size_t i = 0; i < to_copy; i++) {
        buffer[i] = device->rx_buffer[device->rx_tail];
        device->rx_tail = (device->rx_tail + 1) % TSUMIKORO_STM32_RX_BUFFER_SIZE;
        device->rx_count--;
    }

    *received_len = to_copy;

#ifdef TSUMIKORO_HAL_MOCK_DEBUG
    // Debug: print received bytes in hex
    if (to_copy > 0) {
        printf("[HAL_RX] %u bytes:", (unsigned int)to_copy);
        for (size_t i = 0; i < to_copy; i++) {
            printf(" %02X", buffer[i]);
        }
        printf("\n");
    }
#endif

    return TSUMIKORO_HAL_OK;
}

bool tsumikoro_hal_is_bus_idle(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return true;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Bus is busy if transmission in progress
    if (device->tx_active) {
        return false;
    }

    // If no turnaround delay configured, bus is idle
    if (device->turnaround_delay_bytes == 0) {
        return true;
    }

    // Calculate turnaround delay in milliseconds
    // Using 10 bits per byte (1 start + 8 data + 1 stop)
    uint32_t byte_time_us = (10 * 1000000) / device->baud_rate;
    uint32_t turnaround_delay_us = byte_time_us * device->turnaround_delay_bytes;
    uint32_t turnaround_delay_ms = (turnaround_delay_us + 999) / 1000;

    // Check if enough time has passed since last activity
    uint32_t time_since_activity = HAL_GetTick() - device->last_activity_tick;
    return time_since_activity >= turnaround_delay_ms;
}

void tsumikoro_hal_enable_tx(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Assert DE (Driver Enable)
    if (device->stm32_config->de_port != NULL) {
        HAL_GPIO_WritePin(device->stm32_config->de_port,
                         device->stm32_config->de_pin,
                         GPIO_PIN_SET);
    }

    // Keep RE asserted for echo detection (collision detection)
    // If separate RE pin exists and you want to disable RX during TX, uncomment:
    // if (device->stm32_config->re_port != NULL) {
    //     HAL_GPIO_WritePin(device->stm32_config->re_port,
    //                      device->stm32_config->re_pin,
    //                      GPIO_PIN_SET);
    // }
}

void tsumikoro_hal_disable_tx(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // De-assert DE (Driver Enable)
    if (device->stm32_config->de_port != NULL) {
        HAL_GPIO_WritePin(device->stm32_config->de_port,
                         device->stm32_config->de_pin,
                         GPIO_PIN_RESET);
    }

    // Ensure RE is enabled for receiving
    if (device->stm32_config->re_port != NULL) {
        HAL_GPIO_WritePin(device->stm32_config->re_port,
                         device->stm32_config->re_pin,
                         GPIO_PIN_RESET);
    }
}

void tsumikoro_hal_delay_us(uint32_t us)
{
    // STM32 HAL uses millisecond delays
    uint32_t ms = (us + 999) / 1000;
    if (ms > 0) {
        HAL_Delay(ms);
    }
}

uint32_t tsumikoro_hal_get_time_ms(void)
{
    return HAL_GetTick();
}

/* ========== Interrupt Handlers ========== */

void tsumikoro_hal_stm32_uart_irq_handler(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Let HAL process UART interrupts first (this handles TC and updates gState)
    HAL_UART_IRQHandler(&device->uart_handle);

    // After HAL processing, check if TX is complete (gState back to READY)
    if (device->tx_active && device->uart_handle.gState == HAL_UART_STATE_READY) {
        device->tx_active = false;
        device->last_activity_tick = HAL_GetTick();

        // Disable RS-485 driver (DE low) after transmission complete
        if (device->stm32_config->de_port != NULL) {
            HAL_GPIO_WritePin(device->stm32_config->de_port,
                             device->stm32_config->de_pin,
                             GPIO_PIN_RESET);
#ifdef TSUMIKORO_HAL_MOCK_DEBUG
            printf("[HAL_TX] TC complete, DE cleared\n");
#endif
        }
    }

    // Handle RX for non-DMA mode
    if (!device->use_dma && __HAL_UART_GET_FLAG(&device->uart_handle, UART_FLAG_RXNE)) {
        // Receive data byte
        uint8_t data = (uint8_t)(device->uart_handle.Instance->RDR & 0xFF);

        // Store in circular buffer
        if (device->rx_count < TSUMIKORO_STM32_RX_BUFFER_SIZE) {
            device->rx_buffer[device->rx_head] = data;
            device->rx_head = (device->rx_head + 1) % TSUMIKORO_STM32_RX_BUFFER_SIZE;
            device->rx_count++;
            device->last_activity_tick = HAL_GetTick();
        }

        // Clear RXNE flag
        __HAL_UART_CLEAR_FLAG(&device->uart_handle, UART_CLEAR_NEF);
    }
}

void tsumikoro_hal_stm32_dma_tx_irq_handler(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Process DMA TX interrupt
    HAL_DMA_IRQHandler(&device->dma_tx_handle);

    // DMA transfer complete, but UART may still be shifting out last byte
    // Enable TC interrupt to know when UART transmission is fully complete
    __HAL_UART_ENABLE_IT(&device->uart_handle, UART_IT_TC);

    // Note: tx_active will be cleared and DE will be de-asserted in UART IRQ handler
    // when UART_FLAG_TC is set
    device->last_activity_tick = HAL_GetTick();
}

void tsumikoro_hal_stm32_dma_rx_irq_handler(tsumikoro_hal_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    tsumikoro_stm32_device_t *device = (tsumikoro_stm32_device_t *)handle;

    // Process DMA RX interrupt
    HAL_DMA_IRQHandler(&device->dma_rx_handle);

    // Update rx_head based on DMA position for circular mode
    // DMA writes to buffer circularly, we need to track where it's at
    uint32_t dma_write_pos = TSUMIKORO_STM32_RX_BUFFER_SIZE -
                             __HAL_DMA_GET_COUNTER(&device->dma_rx_handle);

    // Calculate how many bytes were received
    size_t old_head = device->rx_head;
    device->rx_head = dma_write_pos;

    if (device->rx_head != old_head) {
        size_t bytes_received;
        if (device->rx_head > old_head) {
            bytes_received = device->rx_head - old_head;
        } else {
            // Wrapped around
            bytes_received = (TSUMIKORO_STM32_RX_BUFFER_SIZE - old_head) + device->rx_head;
        }
        device->rx_count += bytes_received;
        // Prevent overflow (shouldn't happen if bus is processing fast enough)
        if (device->rx_count > TSUMIKORO_STM32_RX_BUFFER_SIZE) {
            device->rx_count = TSUMIKORO_STM32_RX_BUFFER_SIZE;
        }
    }

    device->last_activity_tick = HAL_GetTick();
}
