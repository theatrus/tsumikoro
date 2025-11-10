/**
 * @file main.c
 * @brief NUCLEO-G071RB development board with bus protocol and TMC2130 stepper
 *
 * Development board firmware with bus protocol support and TMC2130 stepper control.
 * - LED blinks on PA5 (LD4) - can be controlled via bus
 * - Button monitored on PC13 (B1) - can be read via bus
 * - USART2 provides debug output via ST-Link VCP (115200 baud)
 * - USART1 provides bus protocol interface (1 Mbaud)
 * - SPI1 controls TMC2130 stepper driver
 *
 * NUCLEO-G071RB pin mapping:
 * - LED (LD4): PA5
 * - User Button (B1): PC13
 * - USART2 (ST-Link VCP): PA2 (TX), PA3 (RX)
 * - USART1 (Bus): PC4 (TX/D1), PC5 (RX/D0), PA1 (DE - RS-485 Driver Enable)
 * - SPI1: PB3 (SCK), PB4 (MISO), PB5 (MOSI)
 * - TMC2130: PB0 (CS), PB1 (STEP), PB2 (DIR), PA0 (EN)
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include "tsumikoro_tmc_hal_stm32.h"
#include "tsumikoro_tmc_motion.h"
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

/* SPI configuration for TMC2130 */
#define TMC_SPI                 SPI1
#define TMC_SPI_CLK_ENABLE()    __HAL_RCC_SPI1_CLK_ENABLE()

/* TMC2130 GPIO pins */
#define TMC_CS_PIN              GPIO_PIN_0
#define TMC_CS_PORT             GPIOB
#define TMC_STEP_PIN            GPIO_PIN_1
#define TMC_STEP_PORT           GPIOB
#define TMC_DIR_PIN             GPIO_PIN_2
#define TMC_DIR_PORT            GPIOB
#define TMC_EN_PIN              GPIO_PIN_0
#define TMC_EN_PORT             GPIOA

/* SPI pins */
#define SPI1_SCK_PIN            GPIO_PIN_3
#define SPI1_SCK_PORT           GPIOB
#define SPI1_MISO_PIN           GPIO_PIN_4
#define SPI1_MISO_PORT          GPIOB
#define SPI1_MOSI_PIN           GPIO_PIN_5
#define SPI1_MOSI_PORT          GPIOB

/* Custom commands for LED/Button control */
#define CMD_LED_SET             0xF001      /**< Set LED state: data[0] = 0/1 */
#define CMD_LED_GET             0xF002      /**< Get LED state */
#define CMD_BUTTON_GET          0xF003      /**< Get button state */

/* Peripheral Handles */
UART_HandleTypeDef huart2;  /* Debug UART (PA2/PA3 - ST-Link VCP) */
SPI_HandleTypeDef hspi1;    /* SPI for TMC2130 */

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

/* TMC2130 stepper driver */
static tsumikoro_tmc_hal_stm32_t g_tmc_instance;
static tsumikoro_tmc_motion_t g_tmc_motion;

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
static void SPI1_Init(void);
static void TMC_Init(void);
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
 * Configure GPIO pins for LED, button, SPI, and TMC2130
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
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

    /* Configure SPI1 pins (PB3=SCK, PB4=MISO, PB5=MOSI) */
    GPIO_InitStruct.Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(SPI1_SCK_PORT, &GPIO_InitStruct);

    /* Configure TMC2130 CS pin (PB0) */
    HAL_GPIO_WritePin(TMC_CS_PORT, TMC_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = TMC_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TMC_CS_PORT, &GPIO_InitStruct);

    /* Configure TMC2130 STEP pin (PB1) */
    HAL_GPIO_WritePin(TMC_STEP_PORT, TMC_STEP_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = TMC_STEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TMC_STEP_PORT, &GPIO_InitStruct);

    /* Configure TMC2130 DIR pin (PB2) */
    HAL_GPIO_WritePin(TMC_DIR_PORT, TMC_DIR_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = TMC_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TMC_DIR_PORT, &GPIO_InitStruct);

    /* Configure TMC2130 EN pin (PA0) - idle high = disabled */
    HAL_GPIO_WritePin(TMC_EN_PORT, TMC_EN_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = TMC_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC_EN_PORT, &GPIO_InitStruct);
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

    /* Initialize UART peripheral via HAL (clocks and GPIO only) */
    if (!tsumikoro_hal_stm32_init_peripheral(&g_stm32_config)) {
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
 * @brief SPI1 Initialization
 * PB3: SCK, PB4: MISO, PB5: MOSI
 * 1 MHz, Mode 3 (CPOL=1, CPHA=1) for TMC2130
 */
static void SPI1_Init(void)
{
    /* Enable SPI clock */
    TMC_SPI_CLK_ENABLE();

    /* Configure SPI */
    hspi1.Instance = TMC_SPI;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;     /* CPOL=1 for TMC */
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;          /* CPHA=1 for TMC */
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  /* 64 MHz / 64 = 1 MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TMC2130 stepper driver initialization
 */
static void TMC_Init(void)
{
    /* TMC hardware configuration */
    tsumikoro_tmc_hal_stm32_config_t tmc_hw_config = {
        .spi_handle = &hspi1,
        .cs_port = TMC_CS_PORT,
        .cs_pin = TMC_CS_PIN,
        .step_port = TMC_STEP_PORT,
        .step_pin = TMC_STEP_PIN,
        .dir_port = TMC_DIR_PORT,
        .dir_pin = TMC_DIR_PIN,
        .en_port = TMC_EN_PORT,
        .en_pin = TMC_EN_PIN
    };

    /* Initialize TMC driver */
    if (!tsumikoro_tmc_hal_stm32_init(&g_tmc_instance, &tmc_hw_config, TMC_CHIP_TMC2130)) {
        Error_Handler();
    }

    /* Get TMC driver handle */
    tsumikoro_tmc_stepper_t *tmc = tsumikoro_tmc_hal_stm32_get_driver(&g_tmc_instance);

    /* Configure TMC2130 parameters */
    tsumikoro_tmc_set_current(tmc, 16, 31, 5);         /* IHOLD=16, IRUN=31, IHOLDDELAY=5 */
    tsumikoro_tmc_set_microsteps(tmc, TMC_MRES_256, true);  /* 256 microsteps with interpolation */
    tsumikoro_tmc_set_stealth_chop(tmc, true);         /* Enable StealthChop for quiet operation */

    /* Initialize motion controller */
    if (!tsumikoro_tmc_motion_init(&g_tmc_motion, tmc)) {
        Error_Handler();
    }

    /* Set motion parameters */
    tsumikoro_tmc_motion_set_params(&g_tmc_motion, 2000, 1000);  /* 2000 steps/sec, 1000 steps/sec^2 */
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

        case TSUMIKORO_CMD_STEPPER_MOVE:
            /* Move to absolute position */
            if (packet->data_len >= 4) {
                int32_t target_position = ((int32_t)packet->data[0] << 24) |
                                         ((int32_t)packet->data[1] << 16) |
                                         ((int32_t)packet->data[2] << 8) |
                                         ((int32_t)packet->data[3]);
                if (tsumikoro_tmc_motion_move_to(&g_tmc_motion, target_position)) {
                    response.data[0] = 0x00;  /* Success */
                    HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] STEPPER_MOVE\r\n", 20, 100);
                } else {
                    response.data[0] = 0xFF;  /* Error */
                }
            } else {
                response.data[0] = 0xFF;  /* Invalid length */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_MOVE_REL:
            /* Move relative to current position */
            if (packet->data_len >= 4) {
                int32_t steps = ((int32_t)packet->data[0] << 24) |
                               ((int32_t)packet->data[1] << 16) |
                               ((int32_t)packet->data[2] << 8) |
                               ((int32_t)packet->data[3]);
                if (tsumikoro_tmc_motion_move_relative(&g_tmc_motion, steps)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error */
                }
            } else {
                response.data[0] = 0xFF;  /* Invalid length */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_STOP:
            /* Stop motor */
            if (tsumikoro_tmc_motion_stop(&g_tmc_motion)) {
                response.data[0] = 0x00;  /* Success */
            } else {
                response.data[0] = 0xFF;  /* Error */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_SET_SPEED:
            /* Set maximum speed */
            if (packet->data_len >= 4) {
                uint32_t max_speed = ((uint32_t)packet->data[0] << 24) |
                                    ((uint32_t)packet->data[1] << 16) |
                                    ((uint32_t)packet->data[2] << 8) |
                                    ((uint32_t)packet->data[3]);
                if (tsumikoro_tmc_motion_set_params(&g_tmc_motion, max_speed, g_tmc_motion.acceleration)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error */
                }
            } else {
                response.data[0] = 0xFF;  /* Invalid length */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_SET_ACCEL:
            /* Set acceleration */
            if (packet->data_len >= 4) {
                uint32_t accel = ((uint32_t)packet->data[0] << 24) |
                                ((uint32_t)packet->data[1] << 16) |
                                ((uint32_t)packet->data[2] << 8) |
                                ((uint32_t)packet->data[3]);
                if (tsumikoro_tmc_motion_set_params(&g_tmc_motion, g_tmc_motion.max_velocity, accel)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error */
                }
            } else {
                response.data[0] = 0xFF;  /* Invalid length */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_HOME:
            /* Set current position to zero */
            if (tsumikoro_tmc_motion_set_position(&g_tmc_motion, 0)) {
                response.data[0] = 0x00;  /* Success */
            } else {
                response.data[0] = 0xFF;  /* Error */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_ENABLE:
            /* Enable motor */
            if (tsumikoro_tmc_motion_enable(&g_tmc_motion, true)) {
                response.data[0] = 0x00;  /* Success */
                HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] STEPPER_ENABLE\r\n", 22, 100);
            } else {
                response.data[0] = 0xFF;  /* Error */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_DISABLE:
            /* Disable motor */
            if (tsumikoro_tmc_motion_enable(&g_tmc_motion, false)) {
                response.data[0] = 0x00;  /* Success */
                HAL_UART_Transmit(&huart2, (uint8_t *)"[BUS] STEPPER_DISABLE\r\n", 23, 100);
            } else {
                response.data[0] = 0xFF;  /* Error */
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_GET_POSITION:
            /* Get current position */
            {
                int32_t position = 0;
                if (tsumikoro_tmc_motion_get_position(&g_tmc_motion, &position)) {
                    response.data[0] = (uint8_t)(position >> 24);
                    response.data[1] = (uint8_t)(position >> 16);
                    response.data[2] = (uint8_t)(position >> 8);
                    response.data[3] = (uint8_t)(position);
                    response.data[4] = tsumikoro_tmc_motion_is_moving(&g_tmc_motion) ? 0x01 : 0x00;
                    response.data_len = 5;
                } else {
                    response.data[0] = 0xFF;  /* Error */
                    response.data_len = 1;
                }
            }
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
    SPI1_Init();
    TMC_Init();
    Bus_Init();

    /* Startup message */
    const char *msg = "\r\n========================================\r\n"
                     "NUCLEO-G071RB Tsumikoro DevBoard\r\n"
                     "STM32G071RBT6 @ 64 MHz\r\n"
                     "128KB Flash, 36KB RAM\r\n"
                     "Bus ID: 0x10, Baud: 1 Mbaud\r\n"
                     "USART1: PC4(TX/D1), PC5(RX/D0), PA1(DE)\r\n"
                     "TMC2130 Stepper: SPI1 + Motion Control\r\n"
                     "========================================\r\n\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);

    uint32_t loop_count = 0;
    uint32_t last_process_time = HAL_GetTick();
    uint32_t last_motion_time_us = 0;

    /* Main loop */
    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t now_us = now * 1000;  /* Convert to microseconds (approximate) */

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

        /* Process motion controller (high frequency for smooth motion) */
        if (now_us - last_motion_time_us >= 100) {  /* Every 100us */
            tsumikoro_tmc_motion_process(&g_tmc_motion, now_us);
            last_motion_time_us = now_us;
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
