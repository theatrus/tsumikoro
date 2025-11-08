/**
 * @file main.c
 * @brief NUCLEO-G071RB Development Board Firmware
 *
 * Simple development board firmware demonstrating basic functionality
 * on the STM32 NUCLEO-G071RB development board.
 *
 * Hardware:
 * - MCU: STM32G071RBT6 (128KB Flash, 36KB RAM)
 * - User LED (LD4): PA5
 * - User Button (B1): PC13
 * - USART2 (ST-Link VCP): PA2 (TX), PA3 (RX)
 */

#include "stm32g0xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Hardware Configuration */
#define LED_PIN         GPIO_PIN_5
#define LED_PORT        GPIOA
#define BUTTON_PIN      GPIO_PIN_13
#define BUTTON_PORT     GPIOC

/* UART Handle */
UART_HandleTypeDef huart2;  /* Debug UART (PA2/PA3 - ST-Link VCP) */

/* LED State */
static bool led_state = false;
static uint32_t led_toggle_time = 0;

/* Button state */
static bool last_button_state = false;

/* Forward declarations */
void Error_Handler(void);

/* ============================================================================
 * System Initialization
 * ============================================================================ */

/**
 * @brief System Clock Configuration
 * Configure the system clock to 64 MHz using HSI16
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initialize the RCC Oscillators according to the specified parameters */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initialize the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization
 * Configure GPIO pins for LED and button
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure LED pin (PA5) */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* Configure Button pin (PC13) */
    GPIO_InitStruct.Pin = BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

/**
 * @brief USART2 Initialization (Debug UART - ST-Link VCP)
 * PA2: TX, PA3: RX
 * 115200 baud, 8N1
 */
static void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins: PA2 (TX), PA3 (RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure UART */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================================================================
 * Main Application
 * ============================================================================ */

int main(void)
{
    /* Initialize HAL */
    HAL_Init();

    /* Configure system clock */
    SystemClock_Config();

    /* Initialize peripherals */
    GPIO_Init();
    USART2_Init();

    /* Startup message */
    const char *msg = "\r\n========================================\r\n"
                     "NUCLEO-G071RB Tsumikoro DevBoard\r\n"
                     "STM32G071RBT6 @ 64 MHz\r\n"
                     "128KB Flash, 36KB RAM\r\n"
                     "========================================\r\n\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    uint32_t loop_count = 0;

    /* Main loop */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* Blink LED every 1000ms */
        if (now - led_toggle_time >= 1000) {
            led_toggle_time = now;
            led_state = !led_state;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

            loop_count++;

            /* Print status every 10 LED toggles (10 seconds) */
            if (loop_count % 10 == 0) {
                char buffer[100];
                int len = snprintf(buffer, sizeof(buffer),
                                 "Uptime: %lu s, LED: %s, Button: %s\r\n",
                                 now / 1000,
                                 led_state ? "ON " : "OFF",
                                 HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET ? "PRESSED" : "RELEASED");
                HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
            }
        }

        /* Check button and send message when pressed */
        bool button_state = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET);
        if (button_state && !last_button_state) {
            /* Button just pressed */
            const char *btn_msg = "Button pressed!\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)btn_msg, strlen(btn_msg), 100);
        }
        last_button_state = button_state;
    }
}

/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Flash LED rapidly */
    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        for (volatile uint32_t i = 0; i < 100000; i++);
    }
}

/**
 * @brief Assert Failed Handler
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the file name and line number */
    Error_Handler();
}
#endif
