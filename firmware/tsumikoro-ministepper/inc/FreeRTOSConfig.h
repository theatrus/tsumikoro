/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS configuration for Tsumikoro ministepper (STM32G071)
 *
 * STM32G071RBT6: Cortex-M0+, 64 MHz, 128KB Flash, 36KB RAM
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ========================================================================== */
/* Core FreeRTOS Configuration                                               */
/* ========================================================================== */

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      64000000  /* 64 MHz */
#define configTICK_RATE_HZ                      1000      /* 1 kHz (1 ms tick) */
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128       /* Words (512 bytes) */
#define configTOTAL_HEAP_SIZE                   (12 * 1024)  /* 12 KB heap for bus threads */
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1

/* ========================================================================== */
/* Co-routine Configuration                                                   */
/* ========================================================================== */

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* ========================================================================== */
/* Software Timer Configuration                                               */
/* ========================================================================== */

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/* ========================================================================== */
/* Optional Functions                                                         */
/* ========================================================================== */

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* ========================================================================== */
/* Cortex-M0+ Specific Configuration                                          */
/* ========================================================================== */

/* Cortex-M0+ doesn't have basepri register, so use primask */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    3

/* Use the system definition for the correct M0 port */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                     __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                     2  /* STM32G0 has 2 priority bits */
#endif

/* Ensure lowest priority for SysTick and PendSV interrupts */
#define configKERNEL_INTERRUPT_PRIORITY         (configPRIO_BITS == 2 ? 3 : 255)

/* ========================================================================== */
/* Interrupt Nesting Configuration                                            */
/* ========================================================================== */

/* Cortex-M0+ doesn't support interrupt masking by priority */
/* All FreeRTOS API calls from ISRs must use ...FromISR() variants */

/* ========================================================================== */
/* FreeRTOS MPU / Memory Protection Configuration                             */
/* ========================================================================== */

#define configENABLE_MPU                        0
#define configENABLE_FPU                        0
#define configENABLE_TRUSTZONE                  0

/* ========================================================================== */
/* Debugging and Statistics                                                   */
/* ========================================================================== */

#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
#define configGENERATE_RUN_TIME_STATS           0

/* ========================================================================== */
/* Assert Configuration                                                       */
/* ========================================================================== */

#define configASSERT(x) if((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

/* ========================================================================== */
/* CMSIS Interrupt Handler Mapping                                           */
/* ========================================================================== */

/* Map FreeRTOS port interrupt handlers to CMSIS standard names */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: xPortSysTickHandler is NOT mapped to SysTick_Handler because
 * we need to call HAL_IncTick() in addition to FreeRTOS tick processing.
 * Instead, we manually call xPortSysTickHandler() from our SysTick_Handler().
 */
/* #define xPortSysTickHandler SysTick_Handler */

#endif /* FREERTOS_CONFIG_H */
