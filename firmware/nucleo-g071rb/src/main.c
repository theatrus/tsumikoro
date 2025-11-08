/**
 * @file main.c
 * @brief NUCLEO-G071RB development board with bus protocol
 *
 * Development board firmware with bus protocol support.
 * - LED blinks on PA5 (LD4) - can be controlled via bus
 * - Button monitored on PC13 (B1) - can be read via bus
 * - USART2 provides debug output via ST-Link VCP (115200 baud)
 * - USART1 provides bus protocol interface (1 Mbaud)
 *
 * NUCLEO-G071RB pin mapping:
 * - LED (LD4): PA5
 * - User Button (B1): PC13
 * - USART2 (ST-Link VCP): PA2 (TX), PA3 (RX)
 * - USART1 (Bus): PC4 (TX/D1), PC5 (RX/D0), PA1 (DE - RS-485 Driver Enable)
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Configuration */
#define DEVICE_ID               0x10        /**< This device's bus ID */
#define BUS_BAUD_RATE           1000000     /**< 1 Mbaud */
#define TURNAROUND_DELAY_BYTES  3           /**< 3 byte intervals turnaround delay */

/* Hardware Configuration */
#define LED_PIN         GPIO_PIN_5
#define LED_PORT        GPIOA
#define BUTTON_PIN      GPIO_PIN_13
#define BUTTON_PORT     GPIOC
#define BUS_DE_PIN      GPIO_PIN_1
#define BUS_DE_PORT     GPIOA

/* UART configuration */
#define BUS_UART                USART1
#define BUS_UART_IRQn           USART1_IRQn
#define BUS_DMA_TX              DMA1_Channel1
#define BUS_DMA_RX              DMA1_Channel2
#define BUS_DMA_TX_IRQn         DMA1_Channel1_IRQn
#define BUS_DMA_RX_IRQn         DMA1_Channel2_3_IRQn

/* Custom commands for LED/Button control */
#define CMD_LED_SET             0xF001      /**< Set LED state: data[0] = 0/1 */
#define CMD_LED_GET             0xF002      /**< Get LED state */
#define CMD_BUTTON_GET          0xF003      /**< Get button state */

/* UART Handles */
UART_HandleTypeDef huart2;  /* Debug UART (PA2/PA3 - ST-Link VCP) */

/* Redirect printf to debug UART for bus protocol debug */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
    return len;
}

/* Bus handles */
static tsumikoro_bus_handle_t g_bus_handle = NULL;
tsumikoro_hal_handle_t g_hal_handle = NULL;  /* Non-static for interrupt handlers */

/* STM32 HAL configuration */
static const tsumikoro_hal_stm32_config_t g_stm32_config = {
    .uart_instance = BUS_UART,
    .dma_tx = BUS_DMA_TX,
    .dma_rx = BUS_DMA_RX,
    .dma_tx_request = DMA_REQUEST_USART1_TX,
    .dma_rx_request = DMA_REQUEST_USART1_RX,
    .de_port = BUS_DE_PORT,  /* PA1 - RS-485 Driver Enable */
    .de_pin = BUS_DE_PIN,
    .re_port = NULL,  /* Shared with DE on most transceivers */
    .re_pin = 0,
    .uart_irq = BUS_UART_IRQn,
    .dma_tx_irq = BUS_DMA_TX_IRQn,
    .dma_rx_irq = BUS_DMA_RX_IRQn
};

/* LED State */
static bool led_state = false;
static bool led_auto_blink = true;  /* Auto-blink enabled by default */
static uint32_t led_toggle_time = 0;

/* Button state */
static bool last_button_state = false;

/* Debug counters from interrupt handlers */
extern volatile uint32_t g_uart1_irq_count;
extern volatile uint32_t g_dma_rx_irq_count;
extern volatile uint32_t g_dma_tx_irq_count;

/* Debug counters for bus processing */
static uint32_t g_bus_process_count = 0;
static uint32_t g_bus_callback_count = 0;
static uint32_t g_last_dma_rx = 0;

/* Forward declarations */
void Error_Handler(void);
static void GPIO_Init(void);
static void USART1_Init(void);
static void USART2_Init(void);
static void Bus_Init(void);
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data);

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

    /* Configure RS-485 Driver Enable pin (PA1) */
    HAL_GPIO_WritePin(BUS_DE_PORT, BUS_DE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BUS_DE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUS_DE_PORT, &GPIO_InitStruct);

    /* Configure UART1 pins (PC4 = TX/D1, PC5 = RX/D0 for bus) */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief USART1 Initialization (Bus UART)
 * PC4: TX (D1), PC5: RX (D0)
 * Initialized by tsumikoro_hal_stm32
 */
static void USART1_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Initialize UART peripheral via HAL */
    if (!tsumikoro_hal_stm32_init_peripheral(&g_stm32_config, BUS_BAUD_RATE)) {
        Error_Handler();
    }
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

/**
 * @brief Bus protocol initialization
 */
static void Bus_Init(void)
{
    /* HAL configuration */
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = BUS_BAUD_RATE,
        .device_id = DEVICE_ID,
        .is_controller = false,  /* This is a peripheral device */
        .turnaround_delay_bytes = TURNAROUND_DELAY_BYTES,
        .platform_data = (void *)&g_stm32_config
    };

    /* Initialize HAL */
    g_hal_handle = tsumikoro_hal_init(&hal_config, NULL, NULL);
    if (g_hal_handle == NULL) {
        Error_Handler();
    }

    /* Bus configuration */
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    bus_config.response_timeout_ms = 50;

    /* Initialize bus handler (bare-metal mode) */
    g_bus_handle = tsumikoro_bus_init(g_hal_handle, &bus_config,
                                       bus_unsolicited_callback, NULL);
    if (g_bus_handle == NULL) {
        Error_Handler();
    }
}

/**
 * @brief Bus unsolicited message callback
 *
 * Called when a message is received that's addressed to this device.
 */
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data)
{
    (void)user_data;
    char debug_buf[80];

    g_bus_callback_count++;

    HAL_UART_Transmit(&huart2, (uint8_t *)"[CALLBACK ENTERED]\r\n", 19, 100);

    if (packet == NULL) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"[CALLBACK] NULL packet\r\n", 24, 100);
        return;
    }

    /* Log received command */
    int len = snprintf(debug_buf, sizeof(debug_buf),
                     "[BUS] RX: dev=0x%02X cmd=0x%04X len=%d\r\n",
                     packet->device_id, packet->command, packet->data_len);
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);

    /* Only process commands addressed to us */
    if (packet->device_id != DEVICE_ID) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"[CALLBACK] Wrong device ID\r\n", 29, 100);
        return;
    }

    /* Prepare response packet */
    tsumikoro_packet_t response = {
        .device_id = 0x00,  /* Send response to controller */
        .command = packet->command,
        .data_len = 0
    };

    /* Handle commands based on command type */
    switch (packet->command) {
        case TSUMIKORO_CMD_PING:
            /* Simple ping response */
            HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] PING\r\n", 12, 100);
            break;

        case TSUMIKORO_CMD_GET_VERSION:
            /* Return firmware version */
            response.data[0] = 1;  /* Major */
            response.data[1] = 0;  /* Minor */
            response.data[2] = 0;  /* Patch */
            response.data[3] = 0;  /* Reserved */
            response.data_len = 4;
            break;

        case TSUMIKORO_CMD_GET_STATUS:
            /* Return general device status */
            response.data[0] = 0x00;  /* Status: OK */
            response.data[1] = led_state ? 0x01 : 0x00;  /* LED state */
            response.data[2] = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) ? 0x01 : 0x00;  /* Button state */
            response.data_len = 3;
            break;

        case CMD_LED_SET:
            /* Set LED state: data[0] = 0 (off), 1 (on), 2 (auto-blink) */
            if (packet->data_len >= 1) {
                if (packet->data[0] == 2) {
                    /* Enable auto-blink mode */
                    led_auto_blink = true;
                    response.data[0] = 0x00;  /* Success */
                    HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] LED Auto-Blink ON\r\n", 25, 100);
                } else {
                    /* Manual control - disable auto-blink */
                    led_auto_blink = false;
                    led_state = (packet->data[0] != 0);
                    HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    response.data[0] = 0x00;  /* Success */
                    len = snprintf(debug_buf, sizeof(debug_buf),
                                 "[BUS] LED Set: %s\r\n", led_state ? "ON" : "OFF");
                    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);
                }
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] LED Set Error\r\n", 21, 100);
            }
            response.data_len = 1;
            break;

        case CMD_LED_GET:
            /* Get LED state */
            response.data[0] = led_state ? 0x01 : 0x00;
            response.data[1] = led_auto_blink ? 0x01 : 0x00;  /* Auto-blink status */
            response.data_len = 2;
            len = snprintf(debug_buf, sizeof(debug_buf),
                         "[BUS] LED Get: state=%d auto=%d\r\n",
                         response.data[0], response.data[1]);
            HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);
            break;

        case CMD_BUTTON_GET:
            /* Get button state */
            response.data[0] = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) ? 0x01 : 0x00;
            response.data_len = 1;
            len = snprintf(debug_buf, sizeof(debug_buf),
                         "[BUS] Button Get: %d\r\n", response.data[0]);
            HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);
            break;

        default:
            /* Unknown command - no response */
            len = snprintf(debug_buf, sizeof(debug_buf),
                         "[BUS] Unknown cmd: 0x%04X\r\n", packet->command);
            HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);
            return;
    }

    /* Send response */
    len = snprintf(debug_buf, sizeof(debug_buf),
                 "[BUS] TX: cmd=0x%04X len=%d\r\n",
                 response.command, response.data_len);
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_buf, len, 100);
    tsumikoro_bus_send_no_response(g_bus_handle, &response);
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
    USART1_Init();
    Bus_Init();

    /* Startup message */
    const char *msg = "\r\n========================================\r\n"
                     "NUCLEO-G071RB Tsumikoro DevBoard\r\n"
                     "STM32G071RBT6 @ 64 MHz\r\n"
                     "128KB Flash, 36KB RAM\r\n"
                     "Bus ID: 0x10, Baud: 1 Mbaud\r\n"
                     "USART1: PC4(TX/D1), PC5(RX/D0), PA1(DE)\r\n"
                     "========================================\r\n\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    uint32_t loop_count = 0;
    uint32_t last_process_time = HAL_GetTick();

    /* Main loop */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* Check for new DMA RX interrupts */
        if (g_dma_rx_irq_count != g_last_dma_rx) {
            char buf[50];
            int len = snprintf(buf, sizeof(buf), "[NEW DMA RX] Count: %lu\r\n", g_dma_rx_irq_count);
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
            g_last_dma_rx = g_dma_rx_irq_count;
        }

        /* Process bus protocol (should be called regularly, ~1ms) */
        if (now - last_process_time >= 1) {
            g_bus_process_count++;
            tsumikoro_bus_process(g_bus_handle);
            last_process_time = now;
        }

        /* Auto-blink LED if enabled */
        if (led_auto_blink && (now - led_toggle_time >= 1000)) {
            led_toggle_time = now;
            led_state = !led_state;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

            loop_count++;

            /* Print status every 10 LED toggles (10 seconds) */
            if (loop_count % 10 == 0) {
                char buffer[200];
                int len = snprintf(buffer, sizeof(buffer),
                                 "Uptime: %lu s, LED: %s\r\n"
                                 "  UART IRQ: %lu, DMA RX: %lu, DMA TX: %lu\r\n"
                                 "  Bus Process: %lu, Callbacks: %lu\r\n",
                                 now / 1000,
                                 led_state ? "ON " : "OFF",
                                 g_uart1_irq_count, g_dma_rx_irq_count, g_dma_tx_irq_count,
                                 g_bus_process_count, g_bus_callback_count);
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
