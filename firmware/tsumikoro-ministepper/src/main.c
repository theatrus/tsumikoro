/**
 * @file main.c
 * @brief Tsumikoro ministepper controller (dual TMC2130 axis driver)
 *
 * Target hardware: hardware/ministepper rev 0.2 (STM32G0B1KBU6 UFQFPN-32, USB).
 * See hardware/ministepper/docs/README.md for the schematic pin map —
 * the `PIN_*` defines below MUST stay in sync with that document.
 *
 * Peripheral allocation:
 *   USART2 (PA2/PA3)          - RS-485 bus @ 1 Mbaud
 *   SPI1   (PA5/PA6/PA7)      - shared SPI to both TMC2130s
 *   TIM1_CH1 (PA8)            - STEP1 pulse generator
 *   TIM2_CH1 (PA0)            - STEP2 pulse generator
 *   I2C1   (PB6/PB7)          - expansion bus (AF6)
 *
 * USART2 is used instead of USART1 to avoid the PA11/PA12 vs PA9/PA10
 * shared-pad trap that hit the servo firmware on STM32G030F6P6 — on
 * the G071 UFQFPN-28 PA2/PA3 are dedicated pins so no SYSCFG remap is
 * needed.
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ===== Configuration ===================================================== */

#define DEVICE_ID               0x01        /**< This device's bus ID (peripheral) */
#define BUS_BAUD_RATE           1000000     /**< 1 Mbaud */
#define TURNAROUND_DELAY_BYTES  3           /**< 3 byte intervals turnaround delay */

/* ===== Pin map (must match hardware/ministepper/docs/README.md) ========== */

/* Status LED - pin 25 */
#define PIN_LED_PORT            GPIOB
#define PIN_LED                 GPIO_PIN_5

/* RS-485 bus (USART2, pins 7-9) */
#define BUS_UART                USART2
#define BUS_UART_IRQn           USART2_IRQn
#define BUS_DMA_TX              DMA1_Channel1
#define BUS_DMA_RX              DMA1_Channel2
#define BUS_DMA_TX_IRQn         DMA1_Channel1_IRQn
#define BUS_DMA_RX_IRQn         DMA1_Channel2_3_IRQn

#define PIN_DE_PORT             GPIOA
#define PIN_DE                  GPIO_PIN_1   /* pin 7  - RS-485 driver enable */
#define PIN_UART_PORT           GPIOA
#define PIN_UART_TX             GPIO_PIN_2   /* pin 8  - USART2 TX */
#define PIN_UART_RX             GPIO_PIN_3   /* pin 9  - USART2 RX */

/* Shared SPI to TMC2130s (SPI1) */
#define PIN_SPI_PORT            GPIOA
#define PIN_SPI_SCK             GPIO_PIN_5   /* pin 11 - SPI1 SCK  (AF0) */
#define PIN_SPI_MISO            GPIO_PIN_6   /* pin 12 - SPI1 MISO (AF0) */
#define PIN_SPI_MOSI            GPIO_PIN_7   /* pin 13 - SPI1 MOSI (AF0) */

/* TMC2130 driver 1 (U2) - axis 1 */
#define PIN_TMC1_CS_PORT        GPIOA
#define PIN_TMC1_CS             GPIO_PIN_4   /* pin 10 - CS1 */
#define PIN_TMC1_STEP_PORT      GPIOA
#define PIN_TMC1_STEP           GPIO_PIN_8   /* pin 16 - STEP1 (TIM1_CH1, AF2) */
#define PIN_TMC1_DIR_PORT       GPIOA
#define PIN_TMC1_DIR            GPIO_PIN_15  /* pin 22 - DIR1 */
#define PIN_TMC1_DIAG_PORT      GPIOB
#define PIN_TMC1_DIAG           GPIO_PIN_1   /* pin 15 - DIAG1 (open-drain + 47k pull-up) */

/* TMC2130 driver 2 (U3) - axis 2 */
#define PIN_TMC2_CS_PORT        GPIOA
#define PIN_TMC2_CS             GPIO_PIN_9   /* pin 19 - CS2 (was PA12, moved for USB) */
#define PIN_TMC2_STEP_PORT      GPIOA
#define PIN_TMC2_STEP           GPIO_PIN_0   /* pin 6  - STEP2 (TIM2_CH1, AF2) */
#define PIN_TMC2_DIR_PORT       GPIOC        /* <-- PC6 (pin 17); PB2 is not bonded out on UFQFPN28 */
#define PIN_TMC2_DIR            GPIO_PIN_6   /* pin 17 - DIR2 */
#define PIN_TMC2_DIAG_PORT      GPIOB
#define PIN_TMC2_DIAG           GPIO_PIN_4   /* pin 24 - DIAG2 */

/* Shared DRV_ENN for both TMC2130s (active-low enable) */
#define PIN_DRV_ENN_PORT        GPIOB
#define PIN_DRV_ENN             GPIO_PIN_2   /* pin 17 - active low enable (was PA11, moved for USB) */

/* Per-axis limit / home switches */
#define PIN_LIMIT1_PORT         GPIOB
#define PIN_LIMIT1              GPIO_PIN_0   /* pin 14 - Limit / home axis 1 */
#define PIN_LIMIT2_PORT         GPIOB
#define PIN_LIMIT2              GPIO_PIN_3   /* pin 23 - Limit / home axis 2 */

/* I2C1 expansion bus (AF6) */
#define PIN_I2C_PORT            GPIOB
#define PIN_I2C_SCL             GPIO_PIN_6   /* pin 26 - I2C1 SCL (AF6) */
#define PIN_I2C_SDA             GPIO_PIN_7   /* pin 27 - I2C1 SDA (AF6) */

/* USB (PA11=D-, PA12=D+ on UFQFPN-32 pins 22/23) — handled by USB peripheral, not GPIO */
/* Spares on UFQFPN32: PB9 (pin 1), PC14 (pin 2), PC15 (pin 3), PA10 (pin 21), PB8 (pin 32) */

/* ===== Task sizes ======================================================== */

#define MAIN_TASK_STACK_SIZE    256

/* ===== Private state ===================================================== */

static tsumikoro_bus_handle_t g_bus_handle = NULL;
static tsumikoro_hal_handle_t g_hal_handle = NULL;

/* STM32 HAL configuration - USART2 + DMAMUX requests for USART2 on DMA1
 * channels 1 (TX) and 2 (RX). */
static const tsumikoro_hal_stm32_config_t g_stm32_config = {
    .uart_instance = BUS_UART,
    .dma_tx = BUS_DMA_TX,
    .dma_rx = BUS_DMA_RX,
    .dma_tx_request = DMA_REQUEST_USART2_TX,
    .dma_rx_request = DMA_REQUEST_USART2_RX,
    .de_port = PIN_DE_PORT,
    .de_pin = PIN_DE,
    .re_port = NULL,            /* ~RE hard-tied to DE on the board */
    .re_pin = 0,
    .uart_irq = BUS_UART_IRQn,
    .dma_tx_irq = BUS_DMA_TX_IRQn,
    .dma_rx_irq = BUS_DMA_RX_IRQn
};

/* Stepper state (minimal — expand when implementing real step generation) */
typedef struct {
    int32_t current_position;
    int32_t target_position;
    bool moving;
    uint16_t speed;
    uint16_t acceleration;
} stepper_state_t;

static stepper_state_t g_axis1 = { .speed = 1000, .acceleration = 500 };
static stepper_state_t g_axis2 = { .speed = 1000, .acceleration = 500 };

/* ===== Forward decls ===================================================== */

void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void Bus_Init(void);
static void Stepper_Init(void);
static void Stepper_Process(void);

static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data);

/* ===== Main task ========================================================= */

static void MainTask(void *argument)
{
    (void)argument;

    GPIO_Init();
    UART_Init();
    Stepper_Init();
    Bus_Init();

    while (1) {
        Stepper_Process();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    BaseType_t result = xTaskCreate(
        MainTask, "Main", MAIN_TASK_STACK_SIZE, NULL,
        tskIDLE_PRIORITY + 1, NULL);

    if (result != pdPASS) {
        Error_Handler();
    }

    vTaskStartScheduler();

    while (1) {
        Error_Handler();
    }
}

/* ===== Peripheral init =================================================== */

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();   /* PC6 = DIR2 */

    GPIO_InitTypeDef gpio = {0};

    /* LED - push-pull output, low speed */
    gpio.Pin = PIN_LED;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_LED_PORT, &gpio);

    /* USART2 AF1 on PA2/PA3 (pins 6/7) */
    gpio.Pin = PIN_UART_TX | PIN_UART_RX;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(PIN_UART_PORT, &gpio);

    /* RS-485 DE on PA1 - the HAL module manages this pin explicitly */
    gpio.Pin = PIN_DE;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PIN_DE_PORT, &gpio);
    HAL_GPIO_WritePin(PIN_DE_PORT, PIN_DE, GPIO_PIN_RESET);  /* start in RX */

    /* TMC1 CS (PA4), TMC2 CS (PA12), DRV_ENN (PA11), DIR1 (PA15) — all GPIO outputs.
     * Hold CS high (inactive) and DRV_ENN high (outputs disabled) at boot. */
    gpio.Pin = PIN_TMC1_CS | PIN_TMC2_CS | PIN_DRV_ENN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, PIN_TMC1_CS | PIN_TMC2_CS, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PIN_DRV_ENN_PORT, PIN_DRV_ENN, GPIO_PIN_SET);  /* drivers off */

    gpio.Pin = PIN_TMC1_DIR;
    HAL_GPIO_Init(PIN_TMC1_DIR_PORT, &gpio);

    gpio.Pin = PIN_TMC2_DIR;
    HAL_GPIO_Init(PIN_TMC2_DIR_PORT, &gpio);

    /* DIAG inputs (open-drain, external 47k pull-ups on board — don't enable internal) */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = PIN_TMC1_DIAG;
    HAL_GPIO_Init(PIN_TMC1_DIAG_PORT, &gpio);
    gpio.Pin = PIN_TMC2_DIAG;
    HAL_GPIO_Init(PIN_TMC2_DIAG_PORT, &gpio);

    /* Limit switches — inputs with internal pull-up (switch closes to GND) */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = PIN_LIMIT1;
    HAL_GPIO_Init(PIN_LIMIT1_PORT, &gpio);
    gpio.Pin = PIN_LIMIT2;
    HAL_GPIO_Init(PIN_LIMIT2_PORT, &gpio);

    /* STEP1 (PA8) will be configured as TIM1_CH1 AF2 by Stepper_Init.
     * STEP2 (PA0) will be configured as TIM2_CH1 AF2 by Stepper_Init.
     * SPI1 (PA5/6/7) will be configured when the SPI driver is added. */
}

static void UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    if (!tsumikoro_hal_stm32_init_peripheral(&g_stm32_config)) {
        Error_Handler();
    }
}

static void Bus_Init(void)
{
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = BUS_BAUD_RATE,
        .device_id = DEVICE_ID,
        .is_controller = false,
        .turnaround_delay_bytes = TURNAROUND_DELAY_BYTES,
        .platform_data = (void *)&g_stm32_config
    };

    g_hal_handle = tsumikoro_hal_init(&hal_config, NULL, NULL);
    if (g_hal_handle == NULL) {
        Error_Handler();
    }

    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    bus_config.response_timeout_ms = 50;

    g_bus_handle = tsumikoro_bus_init(g_hal_handle, &bus_config,
                                       bus_unsolicited_callback, NULL);
    if (g_bus_handle == NULL) {
        Error_Handler();
    }

#if TSUMIKORO_BUS_USE_RTOS
    tsumikoro_hal_deinit(g_hal_handle);
    tsumikoro_hal_rx_callback_t rx_callback = tsumikoro_bus_get_hal_rx_callback();
    g_hal_handle = tsumikoro_hal_init(&hal_config, rx_callback, g_bus_handle);
    if (g_hal_handle == NULL) {
        Error_Handler();
    }
#endif
}

/* ===== Stepper control =================================================== */

static void Stepper_Init(void)
{
    /* TODO:
     *  - Configure SPI1 on PA5/6/7 (AF0) for shared TMC2130 bus
     *  - Configure TIM1_CH1 (PA8) and TIM2_CH1 (PA0) for STEP pulse
     *    generation (one-pulse mode or free-running with DMA-driven ARR)
     *  - Reset TMC2130s (hold DRV_ENN, write GCONF/CHOPCONF/IHOLD_IRUN
     *    via SPI)
     *  - For 0.1 ohm sense resistor variant: leave CHOPCONF.VSENSE = 0
     *  - For 0.68 ohm sense resistor variant (micro-stepper / 28BYJ-48):
     *    set CHOPCONF.VSENSE = 1, use lower IRUN (e.g. 7 for 50 mA)
     */

    g_axis1.current_position = 0;
    g_axis1.target_position = 0;
    g_axis1.moving = false;
    g_axis2.current_position = 0;
    g_axis2.target_position = 0;
    g_axis2.moving = false;
}

static void Stepper_Process(void)
{
    /* TODO: real step generation via TIM1/TIM2 — this is a stub */
    if (g_axis1.moving) {
        if (g_axis1.current_position < g_axis1.target_position)      g_axis1.current_position++;
        else if (g_axis1.current_position > g_axis1.target_position) g_axis1.current_position--;
        else                                                          g_axis1.moving = false;
    }
    if (g_axis2.moving) {
        if (g_axis2.current_position < g_axis2.target_position)      g_axis2.current_position++;
        else if (g_axis2.current_position > g_axis2.target_position) g_axis2.current_position--;
        else                                                          g_axis2.moving = false;
    }
}

/* ===== Bus callback ====================================================== */

static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data)
{
    (void)user_data;

    if (packet == NULL || packet->device_id != DEVICE_ID) {
        return;
    }

    tsumikoro_packet_t response = {
        .device_id = 0x00,
        .command = packet->command,
        .data_len = 0
    };

    switch (packet->command) {
        case TSUMIKORO_CMD_PING:
            break;

        case TSUMIKORO_CMD_GET_VERSION:
            response.data[0] = 1;
            response.data[1] = 0;
            response.data[2] = 0;
            response.data[3] = 0;
            response.data_len = 4;
            break;

        case TSUMIKORO_CMD_GET_STATUS:
            response.data[0] = (g_axis1.moving ? 0x01 : 0x00)
                             | (g_axis2.moving ? 0x02 : 0x00);
            response.data[1] = (uint8_t)(g_axis1.current_position >> 24);
            response.data[2] = (uint8_t)(g_axis1.current_position >> 16);
            response.data[3] = (uint8_t)(g_axis1.current_position >> 8);
            response.data[4] = (uint8_t)(g_axis1.current_position);
            response.data[5] = (uint8_t)(g_axis2.current_position >> 24);
            response.data[6] = (uint8_t)(g_axis2.current_position >> 16);
            response.data[7] = (uint8_t)(g_axis2.current_position >> 8);
            response.data[8] = (uint8_t)(g_axis2.current_position);
            response.data_len = 9;
            break;

        case TSUMIKORO_CMD_STEPPER_MOVE:
            /* byte 0: axis selector (0 = axis 1, 1 = axis 2)
             * bytes 1..4: signed 32-bit absolute target */
            if (packet->data_len >= 5) {
                stepper_state_t *axis = (packet->data[0] == 0) ? &g_axis1 : &g_axis2;
                axis->target_position =
                    ((int32_t)packet->data[1] << 24) |
                    ((int32_t)packet->data[2] << 16) |
                    ((int32_t)packet->data[3] << 8)  |
                    ((int32_t)packet->data[4]);
                axis->moving = true;
                response.data[0] = 0x00;
            } else {
                response.data[0] = 0xFF;
            }
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_STEPPER_STOP:
            /* byte 0: axis selector (0 = axis 1, 1 = axis 2, 0xFF = both) */
            if (packet->data_len >= 1) {
                uint8_t sel = packet->data[0];
                if (sel == 0 || sel == 0xFF) {
                    g_axis1.moving = false;
                    g_axis1.target_position = g_axis1.current_position;
                }
                if (sel == 1 || sel == 0xFF) {
                    g_axis2.moving = false;
                    g_axis2.target_position = g_axis2.current_position;
                }
                response.data[0] = 0x00;
            } else {
                response.data[0] = 0xFF;
            }
            response.data_len = 1;
            break;

        default:
            return;
    }

    tsumikoro_bus_send_no_response(g_bus_handle, &response);
}

/* ===== Clocks + error handler ============================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(PIN_LED_PORT, PIN_LED);
        for (volatile int i = 0; i < 100000; i++);
    }
}

/* ===== Interrupt handlers ================================================ */

void USART2_IRQHandler(void)
{
    tsumikoro_hal_stm32_uart_irq_handler(g_hal_handle);
}

void DMA1_Channel1_IRQHandler(void)
{
    tsumikoro_hal_stm32_dma_tx_irq_handler(g_hal_handle);
}

void DMA1_Channel2_3_IRQHandler(void)
{
    tsumikoro_hal_stm32_dma_rx_irq_handler(g_hal_handle);
}

/* ===== FreeRTOS hooks ==================================================== */

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}
