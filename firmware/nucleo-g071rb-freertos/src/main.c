/**
 * @file main.c
 * @brief NUCLEO-G071RB development board with bus protocol (FreeRTOS version)
 *
 * Development board firmware with bus protocol support using FreeRTOS.
 * - LED blinks on PA5 (LD4) - can be controlled via bus
 * - Button monitored on PC13 (B1) - can be read via bus
 * - USART2 provides debug output via ST-Link VCP (115200 baud)
 * - USART1 provides bus protocol interface (1 Mbaud)
 * - FreeRTOS tasks handle bus processing and LED control
 *
 * NUCLEO-G071RB pin mapping:
 * - LED (LD4): PA5
 * - User Button (B1): PC13
 * - USART2 (ST-Link VCP): PA2 (TX), PA3 (RX)
 * - USART1 (Bus): PC4 (TX/D1), PC5 (RX/D0), PA1 (DE - RS-485 Driver Enable)
 */

#include "stm32g0xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include "tsumikoro_rtos.h"
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

/* Task priorities */
#define PRIORITY_BUS_RX         (tskIDLE_PRIORITY + 3)  /* High priority for RX */
#define PRIORITY_LED_TASK       (tskIDLE_PRIORITY + 1)  /* Low priority for LED */
#define PRIORITY_BUTTON_TASK    (tskIDLE_PRIORITY + 1)  /* Low priority for button */

/* Task stack sizes (in words) */
#define LED_TASK_STACK_SIZE     128
#define BUTTON_TASK_STACK_SIZE  128
#define STATS_TASK_STACK_SIZE   256

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
static SemaphoreHandle_t led_mutex = NULL;

/* Button state */
static bool last_button_state = false;

/* Debug counters from interrupt handlers */
extern volatile uint32_t g_uart1_irq_count;
extern volatile uint32_t g_dma_rx_irq_count;
extern volatile uint32_t g_dma_tx_irq_count;

/* Debug counters for bus processing */
static uint32_t g_bus_callback_count = 0;

/* Forward declarations */
void Error_Handler(void);
static void GPIO_Init(void);
static void USART1_Init(void);
static void USART2_Init(void);
static void Bus_Init(void);
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data);

/* FreeRTOS Task functions */
static void vLEDTask(void *pvParameters);
static void vButtonTask(void *pvParameters);
static void vStatsTask(void *pvParameters);

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
 * @brief Bus protocol initialization (RTOS mode)
 */
static void Bus_Init(void)
{
    printf("[BUS] Starting bus initialization...\r\n");

    /* HAL configuration */
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = BUS_BAUD_RATE,
        .device_id = DEVICE_ID,
        .is_controller = false,  /* This is a peripheral device */
        .turnaround_delay_bytes = TURNAROUND_DELAY_BYTES,
        .platform_data = (void *)&g_stm32_config
    };

    /* Bus configuration */
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    bus_config.response_timeout_ms = 50;

    printf("[BUS] Initializing HAL (no callback)...\r\n");
    /* Initialize HAL without callback first */
    g_hal_handle = tsumikoro_hal_init(&hal_config, NULL, NULL);
    if (g_hal_handle == NULL) {
        printf("[BUS] ERROR: HAL init failed!\r\n");
        Error_Handler();
    }

    printf("[BUS] Heap free before bus init: %u bytes\r\n", xPortGetFreeHeapSize());
    printf("[BUS] Initializing bus handler...\r\n");
    /* Initialize bus handler (RTOS mode) - this creates threads! */
    g_bus_handle = tsumikoro_bus_init(g_hal_handle, &bus_config,
                                       bus_unsolicited_callback, NULL);
    if (g_bus_handle == NULL) {
        printf("[BUS] ERROR: Bus init failed! Heap remaining: %u\r\n", xPortGetFreeHeapSize());
        Error_Handler();
    }
    printf("[BUS] Bus init succeeded! Heap remaining: %u bytes\r\n", xPortGetFreeHeapSize());

    printf("[BUS] Re-initializing HAL with RX callback...\r\n");
    /* In RTOS mode, re-initialize HAL with RX callback */
    /* Note: This is safe because threads are blocked waiting on queues */
    tsumikoro_hal_deinit(g_hal_handle);

    tsumikoro_hal_rx_callback_t rx_callback = tsumikoro_bus_get_hal_rx_callback();
    g_hal_handle = tsumikoro_hal_init(&hal_config, rx_callback, g_bus_handle);
    if (g_hal_handle == NULL) {
        printf("[BUS] ERROR: HAL reinit failed!\r\n");
        Error_Handler();
    }

    printf("[BUS] *** Initialized in RTOS mode ***\r\n");
    printf("[BUS] RX callback registered, threads running\r\n");
}

/**
 * @brief Bus unsolicited message callback
 *
 * Called when a message is received that's addressed to this device.
 * NOTE: This runs in the bus handler thread context!
 */
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data)
{
    (void)user_data;

    g_bus_callback_count++;

    if (packet == NULL) {
        printf("[CALLBACK] NULL packet\r\n");
        return;
    }

    /* Log received command */
    printf("[BUS] RX: dev=0x%02X cmd=0x%04X len=%d\r\n",
           packet->device_id, packet->command, packet->data_len);

    /* Only process commands addressed to us */
    if (packet->device_id != DEVICE_ID) {
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
            printf("[BUS] PING\r\n");
            break;

        case TSUMIKORO_CMD_GET_VERSION:
            response.data[0] = 1;  /* Major */
            response.data[1] = 0;  /* Minor */
            response.data[2] = 0;  /* Patch */
            response.data[3] = 0;  /* Reserved */
            response.data_len = 4;
            break;

        case TSUMIKORO_CMD_GET_STATUS:
            if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                response.data[0] = 0x00;  /* Status: OK */
                response.data[1] = led_state ? 0x01 : 0x00;
                response.data[2] = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) ? 0x01 : 0x00;
                response.data_len = 3;
                xSemaphoreGive(led_mutex);
            }
            break;

        case CMD_LED_SET:
            if (packet->data_len >= 1) {
                if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (packet->data[0] == 2) {
                        led_auto_blink = true;
                        response.data[0] = 0x00;
                        printf("[BUS] LED Auto-Blink ON\r\n");
                    } else {
                        led_auto_blink = false;
                        led_state = (packet->data[0] != 0);
                        HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                        response.data[0] = 0x00;
                        printf("[BUS] LED Set: %s\r\n", led_state ? "ON" : "OFF");
                    }
                    xSemaphoreGive(led_mutex);
                } else {
                    response.data[0] = 0xFF;  /* Error - mutex timeout */
                }
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
            }
            response.data_len = 1;
            break;

        case CMD_LED_GET:
            if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                response.data[0] = led_state ? 0x01 : 0x00;
                response.data[1] = led_auto_blink ? 0x01 : 0x00;
                response.data_len = 2;
                xSemaphoreGive(led_mutex);
            }
            break;

        case CMD_BUTTON_GET:
            response.data[0] = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) ? 0x01 : 0x00;
            response.data_len = 1;
            break;

        default:
            printf("[BUS] Unknown cmd: 0x%04X\r\n", packet->command);
            return;
    }

    /* Send response */
    tsumikoro_bus_send_no_response(g_bus_handle, &response);
}

/* ============================================================================
 * FreeRTOS Tasks
 * ============================================================================ */

/**
 * @brief LED task
 * Handles LED auto-blink mode
 */
static void vLEDTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /* Wait for 1000 ms */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));

        /* Handle auto-blink if enabled */
        if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (led_auto_blink) {
                led_state = !led_state;
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            xSemaphoreGive(led_mutex);
        }
    }
}

/**
 * @brief Button task
 * Monitors button state and reports changes
 */
static void vButtonTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /* Poll every 50 ms */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        bool button_state = (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET);
        if (button_state && !last_button_state) {
            printf("Button pressed!\r\n");
        }
        last_button_state = button_state;
    }
}

/**
 * @brief Statistics task
 * Prints periodic status updates
 */
static void vStatsTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t loop_count = 0;

    printf("[STATS] Stats task started\r\n");

    while (1) {
        /* Print every 10 seconds */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));

        loop_count++;

        /* Get task handles for stack watermark checking */
        TaskHandle_t led_task = xTaskGetHandle("LED");
        TaskHandle_t button_task = xTaskGetHandle("Button");
        TaskHandle_t stats_task = xTaskGetHandle("Stats");
        TaskHandle_t rx_task = xTaskGetHandle("bus_rx");
        TaskHandle_t tx_task = xTaskGetHandle("bus_tx");
        TaskHandle_t handler_task = xTaskGetHandle("bus_handler");

        /* Calculate stack usage percentages (integer math to avoid float printf bloat) */
        uint32_t led_hwm = led_task ? uxTaskGetStackHighWaterMark(led_task) : 0;
        uint32_t button_hwm = button_task ? uxTaskGetStackHighWaterMark(button_task) : 0;
        uint32_t stats_hwm = stats_task ? uxTaskGetStackHighWaterMark(stats_task) : 0;
        uint32_t rx_hwm = rx_task ? uxTaskGetStackHighWaterMark(rx_task) : 0;
        uint32_t tx_hwm = tx_task ? uxTaskGetStackHighWaterMark(tx_task) : 0;
        uint32_t handler_hwm = handler_task ? uxTaskGetStackHighWaterMark(handler_task) : 0;

        /* Print stats using multiple small prints instead of large buffer */
        printf("\r\n=== System Stats (Uptime: %lu s) ===\r\n", xTaskGetTickCount() / 1000);
        printf("LED: %s, Heap free: %u bytes\r\n", led_state ? "ON " : "OFF", xPortGetFreeHeapSize());
        printf("UART IRQ: %lu, DMA RX: %lu, DMA TX: %lu, Callbacks: %lu\r\n",
               g_uart1_irq_count, g_dma_rx_irq_count, g_dma_tx_irq_count, g_bus_callback_count);
        printf("\r\nStack High Water Marks (words free / total):\r\n");
        printf("  LED task:      %3lu / %u  (%lu%% used)\r\n", led_hwm, LED_TASK_STACK_SIZE,
               (LED_TASK_STACK_SIZE - led_hwm) * 100 / LED_TASK_STACK_SIZE);
        printf("  Button task:   %3lu / %u  (%lu%% used)\r\n", button_hwm, BUTTON_TASK_STACK_SIZE,
               (BUTTON_TASK_STACK_SIZE - button_hwm) * 100 / BUTTON_TASK_STACK_SIZE);
        printf("  Stats task:    %3lu / %u  (%lu%% used)\r\n", stats_hwm, STATS_TASK_STACK_SIZE,
               (STATS_TASK_STACK_SIZE - stats_hwm) * 100 / STATS_TASK_STACK_SIZE);
        printf("  bus_rx:        %3lu / %u  (%lu%% used)\r\n", rx_hwm,
               TSUMIKORO_BUS_RX_THREAD_STACK_SIZE / sizeof(StackType_t),
               (TSUMIKORO_BUS_RX_THREAD_STACK_SIZE / sizeof(StackType_t) - rx_hwm) * 100 / (TSUMIKORO_BUS_RX_THREAD_STACK_SIZE / sizeof(StackType_t)));
        printf("  bus_tx:        %3lu / %u  (%lu%% used)\r\n", tx_hwm,
               TSUMIKORO_BUS_TX_THREAD_STACK_SIZE / sizeof(StackType_t),
               (TSUMIKORO_BUS_TX_THREAD_STACK_SIZE / sizeof(StackType_t) - tx_hwm) * 100 / (TSUMIKORO_BUS_TX_THREAD_STACK_SIZE / sizeof(StackType_t)));
        printf("  bus_handler:   %3lu / %u  (%lu%% used)\r\n", handler_hwm,
               TSUMIKORO_BUS_HANDLER_THREAD_STACK_SIZE / sizeof(StackType_t),
               (TSUMIKORO_BUS_HANDLER_THREAD_STACK_SIZE / sizeof(StackType_t) - handler_hwm) * 100 / (TSUMIKORO_BUS_HANDLER_THREAD_STACK_SIZE / sizeof(StackType_t)));
        printf("\r\n");
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
    USART1_Init();
    Bus_Init();

    /* Startup message */
    const char *msg = "\r\n========================================\r\n"
                     "NUCLEO-G071RB Tsumikoro DevBoard\r\n"
                     "STM32G071RBT6 @ 64 MHz\r\n"
                     "128KB Flash, 36KB RAM\r\n"
                     "FreeRTOS v" tskKERNEL_VERSION_NUMBER "\r\n"
                     "Bus ID: 0x10, Baud: 1 Mbaud\r\n"
                     "USART1: PC4(TX/D1), PC5(RX/D0), PA1(DE)\r\n"
                     "RTOS Mode: Enabled\r\n"
                     "========================================\r\n\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    printf("[MAIN] Heap free before task creation: %u bytes\r\n", xPortGetFreeHeapSize());

    /* Create synchronization primitives */
    printf("[MAIN] Creating LED mutex...\r\n");
    led_mutex = xSemaphoreCreateMutex();
    if (led_mutex == NULL) {
        printf("[MAIN] ERROR: Failed to create LED mutex! Heap: %u\r\n", xPortGetFreeHeapSize());
        Error_Handler();
    }

    /* Create tasks */
    BaseType_t result;

    printf("[MAIN] Creating LED task...\r\n");
    result = xTaskCreate(vLEDTask, "LED", LED_TASK_STACK_SIZE, NULL, PRIORITY_LED_TASK, NULL);
    if (result != pdPASS) {
        printf("[MAIN] ERROR: Failed to create LED task! Heap: %u\r\n", xPortGetFreeHeapSize());
        Error_Handler();
    }

    printf("[MAIN] Creating Button task...\r\n");
    result = xTaskCreate(vButtonTask, "Button", BUTTON_TASK_STACK_SIZE, NULL, PRIORITY_BUTTON_TASK, NULL);
    if (result != pdPASS) {
        printf("[MAIN] ERROR: Failed to create Button task! Heap: %u\r\n", xPortGetFreeHeapSize());
        Error_Handler();
    }

    printf("[MAIN] Creating Stats task...\r\n");
    result = xTaskCreate(vStatsTask, "Stats", STATS_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (result != pdPASS) {
        printf("[MAIN] ERROR: Failed to create Stats task! Heap: %u\r\n", xPortGetFreeHeapSize());
        Error_Handler();
    }

    printf("[MAIN] All tasks created successfully!\r\n");
    printf("[MAIN] Heap free before scheduler: %u bytes\r\n", xPortGetFreeHeapSize());

    /* Start scheduler - will create bus protocol tasks internally */
    printf("[MAIN] Starting FreeRTOS scheduler...\r\n");
    vTaskStartScheduler();

    /* Should never reach here */
    Error_Handler();
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
 * @brief Stack overflow hook
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    Error_Handler();
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
