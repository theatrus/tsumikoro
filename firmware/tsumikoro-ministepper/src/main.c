/**
 * @file main.c
 * @brief Tsumikoro ministepper controller with bus communication
 *
 * Example showing how to use the Tsumikoro bus protocol on STM32G071.
 * This peripheral device responds to commands from the bus controller.
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include <string.h>

/* Configuration */
#define DEVICE_ID               0x01        /**< This device's bus ID (peripheral) */
#define BUS_BAUD_RATE           1000000     /**< 1 Mbaud */
#define TURNAROUND_DELAY_BYTES  3           /**< 3 byte intervals turnaround delay */

/* UART configuration (adjust for your hardware) */
#define BUS_UART                USART1
#define BUS_UART_IRQn           USART1_IRQn
#define BUS_DMA_TX              DMA1_Channel1
#define BUS_DMA_RX              DMA1_Channel2
#define BUS_DMA_TX_IRQn         DMA1_Channel1_IRQn
#define BUS_DMA_RX_IRQn         DMA1_Channel2_3_IRQn

/* GPIO for RS-485 transceiver control (adjust for your hardware) */
#define BUS_DE_PORT             GPIOA
#define BUS_DE_PIN              GPIO_PIN_1
#define BUS_RE_PORT             NULL        /**< Optional, can tie RE to DE */
#define BUS_RE_PIN              0

/* Private variables */
static tsumikoro_bus_handle_t g_bus_handle = NULL;
static tsumikoro_hal_handle_t g_hal_handle = NULL;

/* Stepper state */
typedef struct {
    int32_t current_position;
    int32_t target_position;
    bool moving;
    uint16_t speed;
    uint16_t acceleration;
} stepper_state_t;

static stepper_state_t g_stepper_state = {
    .current_position = 0,
    .target_position = 0,
    .moving = false,
    .speed = 1000,
    .acceleration = 500
};

/* Private function prototypes */
void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void Bus_Init(void);
static void Stepper_Init(void);
static void Stepper_Process(void);

/* Bus callback functions */
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data);

/**
 * @brief The application entry point.
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize peripherals */
    GPIO_Init();
    UART_Init();
    Stepper_Init();
    Bus_Init();

    /* Main loop */
    uint32_t last_process_time = HAL_GetTick();
    while (1)
    {
        /* Process bus protocol (should be called regularly, ~1-10ms) */
        if (HAL_GetTick() - last_process_time >= 1) {
            tsumikoro_bus_process(g_bus_handle);
            last_process_time = HAL_GetTick();
        }

        /* Process stepper motor control */
        Stepper_Process();

        /* Optional: Enter low-power mode between processing */
        // __WFI();
    }
}

/**
 * @brief GPIO initialization
 */
static void GPIO_Init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure LED pin (example: PA5) */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_5;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* Configure UART pins (TX=PA9, RX=PA10 for USART1) */
    gpio_init.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* DE/RE pins configured by HAL */
}

/**
 * @brief UART peripheral initialization
 */
static void UART_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure STM32-specific platform data */
    static const tsumikoro_hal_stm32_config_t stm32_config = {
        .uart_instance = BUS_UART,
        .dma_tx = BUS_DMA_TX,
        .dma_rx = BUS_DMA_RX,
        .de_port = BUS_DE_PORT,
        .de_pin = BUS_DE_PIN,
        .re_port = BUS_RE_PORT,
        .re_pin = BUS_RE_PIN,
        .uart_irq = BUS_UART_IRQn,
        .dma_tx_irq = BUS_DMA_TX_IRQn,
        .dma_rx_irq = BUS_DMA_RX_IRQn
    };

    /* Initialize UART peripheral (must be called before HAL init) */
    if (!tsumikoro_hal_stm32_init_peripheral(&stm32_config, BUS_BAUD_RATE)) {
        Error_Handler();
    }
}

/**
 * @brief Bus protocol initialization
 */
static void Bus_Init(void)
{
    /* Configure STM32-specific platform data */
    static const tsumikoro_hal_stm32_config_t stm32_config = {
        .uart_instance = BUS_UART,
        .dma_tx = BUS_DMA_TX,
        .dma_rx = BUS_DMA_RX,
        .de_port = BUS_DE_PORT,
        .de_pin = BUS_DE_PIN,
        .re_port = BUS_RE_PORT,
        .re_pin = BUS_RE_PIN,
        .uart_irq = BUS_UART_IRQn,
        .dma_tx_irq = BUS_DMA_TX_IRQn,
        .dma_rx_irq = BUS_DMA_RX_IRQn
    };

    /* HAL configuration */
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = BUS_BAUD_RATE,
        .device_id = DEVICE_ID,
        .is_controller = false,  /* This is a peripheral device */
        .turnaround_delay_bytes = TURNAROUND_DELAY_BYTES,
        .platform_data = (void *)&stm32_config
    };

    /* Initialize HAL */
    g_hal_handle = tsumikoro_hal_init(&hal_config, NULL, NULL);
    if (g_hal_handle == NULL) {
        Error_Handler();
    }

    /* Bus configuration */
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    bus_config.response_timeout_ms = 50;   /* Respond within 50ms */

    /* Initialize bus handler */
    g_bus_handle = tsumikoro_bus_init(g_hal_handle, &bus_config,
                                       bus_unsolicited_callback, NULL);
    if (g_bus_handle == NULL) {
        Error_Handler();
    }
}

/**
 * @brief Stepper motor initialization
 */
static void Stepper_Init(void)
{
    /* TODO: Initialize stepper motor driver */
    /* - Configure step/direction pins */
    /* - Configure enable pin */
    /* - Set up timer for step generation */

    g_stepper_state.current_position = 0;
    g_stepper_state.target_position = 0;
    g_stepper_state.moving = false;
}

/**
 * @brief Stepper motor processing
 */
static void Stepper_Process(void)
{
    /* TODO: Implement stepper motor control logic */
    /* - Generate step pulses */
    /* - Handle acceleration/deceleration */
    /* - Update position */

    if (g_stepper_state.moving) {
        // Simulate movement
        if (g_stepper_state.current_position < g_stepper_state.target_position) {
            g_stepper_state.current_position++;
        } else if (g_stepper_state.current_position > g_stepper_state.target_position) {
            g_stepper_state.current_position--;
        } else {
            g_stepper_state.moving = false;
        }
    }
}

/**
 * @brief Bus unsolicited message callback
 *
 * Called when a message is received that's addressed to this device.
 * This is where peripheral devices handle commands from the controller.
 */
static void bus_unsolicited_callback(const tsumikoro_packet_t *packet, void *user_data)
{
    (void)user_data;

    if (packet == NULL) {
        return;
    }

    /* Only process commands addressed to us */
    if (packet->device_id != DEVICE_ID) {
        return;
    }

    /* Prepare response packet */
    tsumikoro_packet_t response = {
        .device_id = 0x00,  /* Send response to controller (ID 0x00) */
        .command = packet->command,
        .data_len = 0
    };

    /* Handle commands based on command type */
    switch (packet->command) {
        case TSUMIKORO_CMD_PING:
            /* Simple ping response - no data needed */
            break;

        case TSUMIKORO_CMD_GET_VERSION:
            /* Return firmware version */
            response.data[0] = 1;  /* Major version */
            response.data[1] = 0;  /* Minor version */
            response.data[2] = 0;  /* Patch version */
            response.data[3] = 0;  /* Reserved */
            response.data_len = 4;
            break;

        case TSUMIKORO_CMD_GET_STATUS:
            /* Return device status */
            response.data[0] = g_stepper_state.moving ? 0x01 : 0x00;
            response.data[1] = (uint8_t)(g_stepper_state.current_position >> 24);
            response.data[2] = (uint8_t)(g_stepper_state.current_position >> 16);
            response.data[3] = (uint8_t)(g_stepper_state.current_position >> 8);
            response.data[4] = (uint8_t)(g_stepper_state.current_position);
            response.data_len = 5;
            break;

        case TSUMIKORO_CMD_STEPPER_MOVE:
            /* Move to absolute position */
            if (packet->data_len >= 4) {
                g_stepper_state.target_position =
                    ((int32_t)packet->data[0] << 24) |
                    ((int32_t)packet->data[1] << 16) |
                    ((int32_t)packet->data[2] << 8) |
                    ((int32_t)packet->data[3]);
                g_stepper_state.moving = true;

                /* Acknowledge */
                response.data[0] = 0x00;  /* Success */
                response.data_len = 1;
            } else {
                /* Invalid data length */
                response.data[0] = 0xFF;  /* Error */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_STEPPER_STOP:
            /* Emergency stop */
            g_stepper_state.moving = false;
            g_stepper_state.target_position = g_stepper_state.current_position;

            /* Acknowledge */
            response.data[0] = 0x00;  /* Success */
            response.data_len = 1;
            break;

        default:
            /* Unknown command - send NAK */
            return;
    }

    /* Send response */
    tsumikoro_bus_send_no_response(g_bus_handle, &response);
}

/**
 * @brief System Clock Configuration
 * @note Configured for 64 MHz using HSI and PLL
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the RCC Oscillators according to the specified parameters */
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief This function is executed in case of error occurrence.
 */
static void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Flash LED rapidly */
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for (volatile int i = 0; i < 100000; i++);
    }
}

/**
 * @brief Interrupt handlers
 */

/* UART interrupt handler */
void USART1_IRQHandler(void)
{
    tsumikoro_hal_stm32_uart_irq_handler(g_hal_handle);
}

/* DMA TX interrupt handler */
void DMA1_Channel1_IRQHandler(void)
{
    tsumikoro_hal_stm32_dma_tx_irq_handler(g_hal_handle);
}

/* DMA RX interrupt handler */
void DMA1_Channel2_3_IRQHandler(void)
{
    tsumikoro_hal_stm32_dma_rx_irq_handler(g_hal_handle);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the file name and line number */
    (void)file;
    (void)line;
}
#endif /* USE_FULL_ASSERT */
