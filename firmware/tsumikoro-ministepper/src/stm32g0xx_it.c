/**
 * @file stm32g0xx_it.c
 * @brief Interrupt Service Routines for Tsumikoro ministepper
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stm32g0xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief SysTick interrupt handler
 *
 * This handler must call both HAL_IncTick() for STM32 HAL and
 * xPortSysTickHandler() for FreeRTOS when the scheduler is running.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    /* Call FreeRTOS tick handler if scheduler has started */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        extern void xPortSysTickHandler(void);
        xPortSysTickHandler();
    }
}

/**
 * @brief SVC interrupt handler
 *
 * This is mapped to vPortSVCHandler() via FreeRTOSConfig.h
 * The FreeRTOS port provides the actual implementation.
 */
void SVC_Handler(void)
{
    /* FreeRTOS port provides implementation via vPortSVCHandler macro */
}

/**
 * @brief PendSV interrupt handler
 *
 * This is mapped to xPortPendSVHandler() via FreeRTOSConfig.h
 * The FreeRTOS port provides the actual implementation.
 */
void PendSV_Handler(void)
{
    /* FreeRTOS port provides implementation via xPortPendSVHandler macro */
}

/**
 * @brief Hard fault handler
 */
void HardFault_Handler(void)
{
    while (1) {
        /* Trap */
    }
}
