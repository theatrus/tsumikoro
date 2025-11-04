/**
 * @file tsumikoro_rtos.c
 * @brief RTOS abstraction layer implementation
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_rtos.h"

#if TSUMIKORO_BUS_USE_RTOS

/* ========== FreeRTOS Implementation (ESP32 and STM32) ========== */

#ifdef TSUMIKORO_RTOS_FREERTOS

tsumikoro_queue_t tsumikoro_queue_create(uint32_t item_count, uint32_t item_size)
{
    return (tsumikoro_queue_t)xQueueCreate(item_count, item_size);
}

void tsumikoro_queue_delete(tsumikoro_queue_t queue)
{
    if (queue) {
        vQueueDelete((QueueHandle_t)queue);
    }
}

bool tsumikoro_queue_send(tsumikoro_queue_t queue, const void *item, uint32_t timeout_ms)
{
    if (!queue || !item) return false;

    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueSend((QueueHandle_t)queue, item, ticks) == pdTRUE;
}

bool tsumikoro_queue_send_from_isr(tsumikoro_queue_t queue, const void *item)
{
    if (!queue || !item) return false;

    BaseType_t higher_priority_woken = pdFALSE;
    BaseType_t result = xQueueSendFromISR((QueueHandle_t)queue, item, &higher_priority_woken);

#ifdef TSUMIKORO_RTOS_STM32
    portYIELD_FROM_ISR(higher_priority_woken);
#else
    if (higher_priority_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
#endif

    return result == pdTRUE;
}

bool tsumikoro_queue_receive(tsumikoro_queue_t queue, void *item, uint32_t timeout_ms)
{
    if (!queue || !item) return false;

    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive((QueueHandle_t)queue, item, ticks) == pdTRUE;
}

uint32_t tsumikoro_queue_count(tsumikoro_queue_t queue)
{
    if (!queue) return 0;
    return (uint32_t)uxQueueMessagesWaiting((QueueHandle_t)queue);
}

tsumikoro_semaphore_t tsumikoro_semaphore_create_binary(void)
{
    return (tsumikoro_semaphore_t)xSemaphoreCreateBinary();
}

void tsumikoro_semaphore_delete(tsumikoro_semaphore_t sem)
{
    if (sem) {
        vSemaphoreDelete((SemaphoreHandle_t)sem);
    }
}

bool tsumikoro_semaphore_wait(tsumikoro_semaphore_t sem, uint32_t timeout_ms)
{
    if (!sem) return false;

    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake((SemaphoreHandle_t)sem, ticks) == pdTRUE;
}

bool tsumikoro_semaphore_signal(tsumikoro_semaphore_t sem)
{
    if (!sem) return false;
    return xSemaphoreGive((SemaphoreHandle_t)sem) == pdTRUE;
}

bool tsumikoro_semaphore_signal_from_isr(tsumikoro_semaphore_t sem)
{
    if (!sem) return false;

    BaseType_t higher_priority_woken = pdFALSE;
    BaseType_t result = xSemaphoreGiveFromISR((SemaphoreHandle_t)sem, &higher_priority_woken);

#ifdef TSUMIKORO_RTOS_STM32
    portYIELD_FROM_ISR(higher_priority_woken);
#else
    if (higher_priority_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
#endif

    return result == pdTRUE;
}

static void thread_wrapper(void *arg)
{
    tsumikoro_thread_config_t *config = (tsumikoro_thread_config_t *)arg;
    if (config && config->function) {
        config->function(config->argument);
    }
    vTaskDelete(NULL);  // Delete self if function returns
}

tsumikoro_thread_t tsumikoro_thread_create(const tsumikoro_thread_config_t *config)
{
    if (!config || !config->function) return NULL;

    // Map priority to FreeRTOS priority
    UBaseType_t priority;
    switch (config->priority) {
        case TSUMIKORO_THREAD_PRIORITY_LOW:      priority = 1; break;
        case TSUMIKORO_THREAD_PRIORITY_NORMAL:   priority = 2; break;
        case TSUMIKORO_THREAD_PRIORITY_HIGH:     priority = 3; break;
        case TSUMIKORO_THREAD_PRIORITY_REALTIME: priority = 4; break;
        default: priority = 2; break;
    }

    TaskHandle_t handle = NULL;
    BaseType_t result = xTaskCreate(
        thread_wrapper,
        config->name ? config->name : "tsumikoro",
        config->stack_size / sizeof(StackType_t),
        (void *)config,
        priority,
        &handle
    );

    return (result == pdPASS) ? (tsumikoro_thread_t)handle : NULL;
}

void tsumikoro_thread_delete(tsumikoro_thread_t thread)
{
    vTaskDelete((TaskHandle_t)thread);
}

void tsumikoro_thread_sleep_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void tsumikoro_thread_yield(void)
{
    taskYIELD();
}

tsumikoro_thread_t tsumikoro_thread_get_current(void)
{
    return (tsumikoro_thread_t)xTaskGetCurrentTaskHandle();
}

uint32_t tsumikoro_critical_enter(void)
{
    taskENTER_CRITICAL();
    return 0;  // FreeRTOS doesn't return state
}

void tsumikoro_critical_exit(uint32_t state)
{
    (void)state;
    taskEXIT_CRITICAL();
}

bool tsumikoro_task_notify_wait(uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return ulTaskNotifyTake(pdTRUE, ticks) != 0;
}

bool tsumikoro_task_notify(tsumikoro_thread_t thread)
{
    if (!thread) return false;
    return xTaskNotifyGive((TaskHandle_t)thread) == pdPASS;
}

bool tsumikoro_task_notify_from_isr(tsumikoro_thread_t thread)
{
    if (!thread) return false;

    BaseType_t higher_priority_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)thread, &higher_priority_woken);

#ifdef TSUMIKORO_RTOS_STM32
    portYIELD_FROM_ISR(higher_priority_woken);
#else
    if (higher_priority_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
#endif

    return true;
}

#endif /* TSUMIKORO_RTOS_FREERTOS */

#endif /* TSUMIKORO_BUS_USE_RTOS */
