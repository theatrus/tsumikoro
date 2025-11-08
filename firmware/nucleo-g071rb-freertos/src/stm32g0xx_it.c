/**
 * @file stm32g0xx_it.c
 * @brief Interrupt Service Routines
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_hal_stm32.h"
#include "FreeRTOS.h"
#include "task.h"

/* External UART handles */
extern UART_HandleTypeDef huart2;

/* External bus HAL handle */
extern tsumikoro_hal_handle_t g_hal_handle;

/* Debug counters for interrupt activity */
volatile uint32_t g_uart1_irq_count = 0;
volatile uint32_t g_dma_rx_irq_count = 0;
volatile uint32_t g_dma_tx_irq_count = 0;

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    while (1) {
    }
}

/*
 * Note: SVC_Handler, PendSV_Handler, and SysTick_Handler are provided by
 * FreeRTOS port (ARM_CM0/port.c) via macros in FreeRTOSConfig.h:
 *   #define vPortSVCHandler    SVC_Handler
 *   #define xPortPendSVHandler PendSV_Handler
 *   #define xPortSysTickHandler SysTick_Handler
 *
 * For HAL compatibility, we need to hook HAL_IncTick() into SysTick:
 */

/**
 * @brief Override weak SysTick_Handler to add HAL tick
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    /* Call FreeRTOS tick handler (from port.c) */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        extern void xPortSysTickHandler(void);
        xPortSysTickHandler();
    }
}

/**
 * @brief This function handles USART1 global interrupt (Bus UART).
 */
void USART1_IRQHandler(void)
{
    g_uart1_irq_count++;
    tsumikoro_hal_stm32_uart_irq_handler(g_hal_handle);
}

/**
 * @brief This function handles USART2 global interrupt (Debug UART).
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief This function handles DMA1 Channel 1 interrupt (Bus UART TX).
 */
void DMA1_Channel1_IRQHandler(void)
{
    g_dma_tx_irq_count++;
    tsumikoro_hal_stm32_dma_tx_irq_handler(g_hal_handle);
}

/**
 * @brief This function handles DMA1 Channel 2 and Channel 3 interrupt.
 * Channel 2 is used for Bus UART RX.
 */
void DMA1_Channel2_3_IRQHandler(void)
{
    g_dma_rx_irq_count++;
    tsumikoro_hal_stm32_dma_rx_irq_handler(g_hal_handle);
}
