/**
 * @file tsumikoro_rtos.h
 * @brief RTOS abstraction layer for Tsumikoro bus
 *
 * Provides platform-independent RTOS primitives (queues, semaphores, threads)
 * for FreeRTOS implementations on STM32 and ESP32.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_RTOS_H
#define TSUMIKORO_RTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Configuration ========== */

/**
 * @brief Enable RTOS support
 *
 * Define TSUMIKORO_BUS_USE_RTOS=1 to enable FreeRTOS threading.
 * When disabled, bus runs in bare-metal polling mode.
 */
#ifndef TSUMIKORO_BUS_USE_RTOS
#define TSUMIKORO_BUS_USE_RTOS 0
#endif

#if TSUMIKORO_BUS_USE_RTOS

/* ========== Platform Detection ========== */

#if defined(ESP_PLATFORM) || defined(ESP32)
    /* ESP32 with ESP-IDF FreeRTOS */
    #define TSUMIKORO_RTOS_FREERTOS
    #define TSUMIKORO_RTOS_ESP32
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "freertos/semphr.h"
#elif defined(STM32G0) || defined(STM32F4) || defined(STM32H7) || defined(STM32)
    /* STM32 with native FreeRTOS */
    #define TSUMIKORO_RTOS_FREERTOS
    #define TSUMIKORO_RTOS_STM32
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "semphr.h"
#else
    #error "Unknown RTOS platform. Define STM32 or ESP_PLATFORM."
#endif

/* ========== Type Definitions ========== */

/**
 * @brief Opaque queue handle
 */
typedef void* tsumikoro_queue_t;

/**
 * @brief Opaque semaphore handle
 */
typedef void* tsumikoro_semaphore_t;

/**
 * @brief Opaque thread handle
 */
typedef void* tsumikoro_thread_t;

/**
 * @brief Thread function prototype
 */
typedef void (*tsumikoro_thread_func_t)(void *arg);

/**
 * @brief Thread priority levels
 */
typedef enum {
    TSUMIKORO_THREAD_PRIORITY_LOW = 0,
    TSUMIKORO_THREAD_PRIORITY_NORMAL = 1,
    TSUMIKORO_THREAD_PRIORITY_HIGH = 2,
    TSUMIKORO_THREAD_PRIORITY_REALTIME = 3
} tsumikoro_thread_priority_t;

/**
 * @brief Thread configuration
 */
typedef struct {
    const char *name;                      /**< Thread name (for debugging) */
    tsumikoro_thread_func_t function;      /**< Thread entry function */
    void *argument;                        /**< Argument passed to function */
    uint32_t stack_size;                   /**< Stack size in bytes */
    tsumikoro_thread_priority_t priority;  /**< Thread priority */
} tsumikoro_thread_config_t;

/* ========== Queue API ========== */

/**
 * @brief Create a queue
 *
 * @param item_count Maximum number of items in queue
 * @param item_size Size of each item in bytes
 * @return Queue handle on success, NULL on failure
 */
tsumikoro_queue_t tsumikoro_queue_create(uint32_t item_count, uint32_t item_size);

/**
 * @brief Delete a queue
 *
 * @param queue Queue handle
 */
void tsumikoro_queue_delete(tsumikoro_queue_t queue);

/**
 * @brief Send item to queue (non-blocking)
 *
 * @param queue Queue handle
 * @param item Pointer to item to send
 * @param timeout_ms Timeout in milliseconds (0 = no wait, UINT32_MAX = infinite)
 * @return true if sent successfully, false if queue full or timeout
 */
bool tsumikoro_queue_send(tsumikoro_queue_t queue, const void *item, uint32_t timeout_ms);

/**
 * @brief Send item to queue from ISR
 *
 * @param queue Queue handle
 * @param item Pointer to item to send
 * @return true if sent successfully, false if queue full
 */
bool tsumikoro_queue_send_from_isr(tsumikoro_queue_t queue, const void *item);

/**
 * @brief Receive item from queue (blocking)
 *
 * @param queue Queue handle
 * @param item Pointer to buffer for received item
 * @param timeout_ms Timeout in milliseconds (0 = no wait, UINT32_MAX = infinite)
 * @return true if received successfully, false if queue empty or timeout
 */
bool tsumikoro_queue_receive(tsumikoro_queue_t queue, void *item, uint32_t timeout_ms);

/**
 * @brief Get number of items in queue
 *
 * @param queue Queue handle
 * @return Number of items currently in queue
 */
uint32_t tsumikoro_queue_count(tsumikoro_queue_t queue);

/* ========== Semaphore API ========== */

/**
 * @brief Create a binary semaphore
 *
 * @return Semaphore handle on success, NULL on failure
 */
tsumikoro_semaphore_t tsumikoro_semaphore_create_binary(void);

/**
 * @brief Delete a semaphore
 *
 * @param sem Semaphore handle
 */
void tsumikoro_semaphore_delete(tsumikoro_semaphore_t sem);

/**
 * @brief Wait for semaphore (blocking)
 *
 * @param sem Semaphore handle
 * @param timeout_ms Timeout in milliseconds (0 = no wait, UINT32_MAX = infinite)
 * @return true if acquired, false on timeout
 */
bool tsumikoro_semaphore_wait(tsumikoro_semaphore_t sem, uint32_t timeout_ms);

/**
 * @brief Signal semaphore (give)
 *
 * @param sem Semaphore handle
 * @return true on success
 */
bool tsumikoro_semaphore_signal(tsumikoro_semaphore_t sem);

/**
 * @brief Signal semaphore from ISR
 *
 * @param sem Semaphore handle
 * @return true on success
 */
bool tsumikoro_semaphore_signal_from_isr(tsumikoro_semaphore_t sem);

/* ========== Thread API ========== */

/**
 * @brief Create and start a thread
 *
 * @param config Thread configuration
 * @return Thread handle on success, NULL on failure
 */
tsumikoro_thread_t tsumikoro_thread_create(const tsumikoro_thread_config_t *config);

/**
 * @brief Delete a thread
 *
 * Note: Not all RTOS implementations support deleting other threads.
 * This may only work for deleting the current thread.
 *
 * @param thread Thread handle (NULL = current thread)
 */
void tsumikoro_thread_delete(tsumikoro_thread_t thread);

/**
 * @brief Sleep current thread
 *
 * @param ms Milliseconds to sleep
 */
void tsumikoro_thread_sleep_ms(uint32_t ms);

/**
 * @brief Yield current thread
 */
void tsumikoro_thread_yield(void);

/**
 * @brief Get current thread handle
 *
 * @return Current thread handle
 */
tsumikoro_thread_t tsumikoro_thread_get_current(void);

/* ========== Critical Section API ========== */

/**
 * @brief Enter critical section (disable interrupts)
 *
 * Must be paired with tsumikoro_critical_exit().
 * Critical sections should be kept as short as possible.
 *
 * @return State value to pass to tsumikoro_critical_exit()
 */
uint32_t tsumikoro_critical_enter(void);

/**
 * @brief Exit critical section (restore interrupts)
 *
 * @param state State value from tsumikoro_critical_enter()
 */
void tsumikoro_critical_exit(uint32_t state);

/* ========== Task Notification API ========== */

/**
 * @brief Wait for task notification
 *
 * Clears notification value on return.
 *
 * @param timeout_ms Timeout in milliseconds (UINT32_MAX = infinite)
 * @return true if notification received, false on timeout
 */
bool tsumikoro_task_notify_wait(uint32_t timeout_ms);

/**
 * @brief Send task notification
 *
 * @param thread Target thread handle
 * @return true on success
 */
bool tsumikoro_task_notify(tsumikoro_thread_t thread);

/**
 * @brief Send task notification from ISR
 *
 * @param thread Target thread handle
 * @return true on success
 */
bool tsumikoro_task_notify_from_isr(tsumikoro_thread_t thread);

#endif /* TSUMIKORO_BUS_USE_RTOS */

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_RTOS_H */
