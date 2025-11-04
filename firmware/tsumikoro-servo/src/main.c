/**
 * @file main.c
 * @brief Tsumikoro servo controller with bus communication (bare-metal)
 *
 * Example showing bare-metal bus protocol usage on STM32G030.
 * This peripheral device responds to commands from the bus controller
 * using polling mode (no RTOS, minimal RAM footprint).
 */

#include "stm32g0xx_hal.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_stm32.h"
#include "servo_pwm.h"
#include "motor_hbridge.h"
#include "limit_switch.h"
#include <string.h>

/* Configuration */
#define DEVICE_ID               0x02        /**< This device's bus ID (peripheral) */
#define BUS_BAUD_RATE           1000000     /**< 1 Mbaud */
#define TURNAROUND_DELAY_BYTES  3           /**< 3 byte intervals turnaround delay */

/* UART configuration (adjust for your hardware) */
#define BUS_UART                USART1
#define BUS_UART_IRQn           USART1_IRQn
#define BUS_DMA_TX              DMA1_Channel1
#define BUS_DMA_RX              DMA1_Channel2
#define BUS_DMA_TX_IRQn         DMA1_Channel1_IRQn
#define BUS_DMA_RX_IRQn         DMA1_Channel2_3_IRQn

/* GPIO for RS-485 transceiver control */
#define BUS_DE_PORT             GPIOA
#define BUS_DE_PIN              GPIO_PIN_1
#define BUS_RE_PORT             NULL
#define BUS_RE_PIN              0

/* Private variables */
static tsumikoro_bus_handle_t g_bus_handle = NULL;
static tsumikoro_hal_handle_t g_hal_handle = NULL;

/* STM32 HAL configuration */
static const tsumikoro_hal_stm32_config_t g_stm32_config = {
    .uart_instance = BUS_UART,
    .dma_tx = BUS_DMA_TX,
    .dma_rx = BUS_DMA_RX,
    .dma_tx_request = DMA_REQUEST_USART1_TX,
    .dma_rx_request = DMA_REQUEST_USART1_RX,
    .de_port = BUS_DE_PORT,
    .de_pin = BUS_DE_PIN,
    .re_port = BUS_RE_PORT,
    .re_pin = BUS_RE_PIN,
    .uart_irq = BUS_UART_IRQn,
    .dma_tx_irq = BUS_DMA_TX_IRQn,
    .dma_rx_irq = BUS_DMA_RX_IRQn
};

/* Servo state - moved to servo_pwm module */

/* Private function prototypes */
void SystemClock_Config(void);
static void Error_Handler(void);
static void GPIO_Init(void);
static void UART_Init(void);
static void Bus_Init(void);
static void Servo_Init(void);
static void Servo_Process(void);
static void Motor_Init(void);
static void LimitSwitch_Init(void);

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
    Servo_Init();
    Motor_Init();
    LimitSwitch_Init();
    Bus_Init();

    /* Main loop - bare-metal polling */
    uint32_t last_process_time = HAL_GetTick();
    while (1)
    {
        /* Process bus protocol (should be called regularly, ~1ms) */
        if (HAL_GetTick() - last_process_time >= 1) {
            tsumikoro_bus_process(g_bus_handle);
            last_process_time = HAL_GetTick();
        }

        /* Process servo motor control */
        Servo_Process();
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
}

/**
 * @brief UART peripheral initialization
 */
static void UART_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Initialize UART peripheral */
    if (!tsumikoro_hal_stm32_init_peripheral(&g_stm32_config, BUS_BAUD_RATE)) {
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
 * @brief Servo motor initialization
 */
static void Servo_Init(void)
{
    /* Initialize servo PWM system using LL drivers */
    if (!servo_pwm_init()) {
        Error_Handler();
    }
}

/**
 * @brief Servo motor processing
 */
static void Servo_Process(void)
{
    /* Process servo movement - updates PWM outputs */
    servo_pwm_process();
}

/**
 * @brief Motor H-bridge initialization
 */
static void Motor_Init(void)
{
    /* Initialize H-bridge motor driver using LL drivers */
    if (!motor_hbridge_init()) {
        Error_Handler();
    }
}

/**
 * @brief Limit switch initialization
 */
static void LimitSwitch_Init(void)
{
    /* Initialize limit switch input using LL drivers */
    if (!limit_switch_init()) {
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

    if (packet == NULL) {
        return;
    }

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
            /* Simple ping response */
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
            /* Return general device status (not servo-specific) */
            response.data[0] = 0x00;  /* Status: OK */
            response.data[1] = servo_pwm_get_channel_count();  /* Number of servo channels */
            response.data_len = 2;
            break;

        case TSUMIKORO_CMD_GET_LIMIT_SWITCH:
            /* Get limit switch state
             * Response: [triggered, raw_state] */
            response.data[0] = limit_switch_is_triggered() ? 0x01 : 0x00;
            response.data[1] = limit_switch_get_raw_state() ? 0x01 : 0x00;
            response.data_len = 2;
            break;

        case TSUMIKORO_CMD_SERVO_SET_POSITION:
            /* Set servo position (0-1800 = 0-180 degrees * 10)
             * Data: [channel_index, position_hi, position_lo] */
            if (packet->data_len >= 3) {
                uint8_t channel = packet->data[0];
                uint16_t position = ((uint16_t)packet->data[1] << 8) |
                                   ((uint16_t)packet->data[2]);

                if (servo_pwm_set_position(channel, position)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid channel/position */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_GET_POSITION:
            /* Get current servo position
             * Data: [channel_index] */
            if (packet->data_len >= 1) {
                uint8_t channel = packet->data[0];
                uint16_t position = servo_pwm_get_position(channel);
                response.data[0] = (uint8_t)(position >> 8);
                response.data[1] = (uint8_t)(position);
                response.data_len = 2;
            } else {
                response.data[0] = 0xFF;  /* Error - missing channel index */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_SET_SPEED:
            /* Set servo movement speed
             * Data: [channel_index, speed_hi, speed_lo] */
            if (packet->data_len >= 3) {
                uint8_t channel = packet->data[0];
                uint16_t speed = ((uint16_t)packet->data[1] << 8) |
                                ((uint16_t)packet->data[2]);

                if (servo_pwm_set_speed(channel, speed)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid channel */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_CALIBRATE:
            /* Calibrate servo endpoints
             * Data: [channel_index, min_pulse_hi, min_pulse_lo, max_pulse_hi, max_pulse_lo] */
            if (packet->data_len >= 5) {
                uint8_t channel = packet->data[0];
                uint16_t min_pulse_us = ((uint16_t)packet->data[1] << 8) |
                                       ((uint16_t)packet->data[2]);
                uint16_t max_pulse_us = ((uint16_t)packet->data[3] << 8) |
                                       ((uint16_t)packet->data[4]);

                if (servo_pwm_set_calibration(channel, min_pulse_us, max_pulse_us)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid parameters */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_ENABLE:
            /* Enable/disable servo output
             * Data: [channel_index, enable (0=disable, 1=enable)] */
            if (packet->data_len >= 2) {
                uint8_t channel = packet->data[0];
                bool enable = (packet->data[1] != 0);

                if (servo_pwm_enable(channel, enable)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid channel */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_SET_MULTI:
            /* Set multiple servo positions
             * Data: [count, ch0_idx, pos0_hi, pos0_lo, ch1_idx, pos1_hi, pos1_lo, ...] */
            if (packet->data_len >= 1) {
                uint8_t count = packet->data[0];
                uint8_t idx = 1;
                uint8_t success_count = 0;

                for (uint8_t i = 0; i < count && idx + 3 <= packet->data_len; i++) {
                    uint8_t channel = packet->data[idx];
                    uint16_t position = ((uint16_t)packet->data[idx + 1] << 8) |
                                       ((uint16_t)packet->data[idx + 2]);
                    idx += 3;

                    if (servo_pwm_set_position(channel, position)) {
                        success_count++;
                    }
                }

                response.data[0] = success_count;  /* Number of successful updates */
                response.data_len = 1;
            } else {
                response.data[0] = 0x00;  /* No servos updated */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_SERVO_GET_STATUS:
            /* Get servo channel status
             * Data: [channel_index]
             * Response: [enabled, moving, current_pos_hi, current_pos_lo, target_pos_hi, target_pos_lo] */
            if (packet->data_len >= 1) {
                uint8_t channel = packet->data[0];

                if (channel < servo_pwm_get_channel_count()) {
                    uint16_t current_pos = servo_pwm_get_position(channel);
                    uint16_t target_pos = servo_pwm_get_target_position(channel);
                    bool moving = servo_pwm_is_moving(channel);

                    response.data[0] = 0x01;  /* Enabled (assume enabled if valid channel) */
                    response.data[1] = moving ? 0x01 : 0x00;
                    response.data[2] = (uint8_t)(current_pos >> 8);
                    response.data[3] = (uint8_t)(current_pos);
                    response.data[4] = (uint8_t)(target_pos >> 8);
                    response.data[5] = (uint8_t)(target_pos);
                    response.data_len = 6;
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid channel */
                    response.data_len = 1;
                }
            } else {
                response.data[0] = 0xFF;  /* Error - missing channel index */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_DCMOTOR_SET:
            /* Set motor speed and direction
             * Data: [speed_hi, speed_lo, direction] */
            if (packet->data_len >= 3) {
                uint16_t speed = ((uint16_t)packet->data[0] << 8) |
                                ((uint16_t)packet->data[1]);
                motor_direction_t direction = (motor_direction_t)packet->data[2];

                if (motor_hbridge_set(speed, direction)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error - invalid parameters */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_DCMOTOR_GET_SPEED:
            /* Get current motor speed
             * Response: [speed_hi, speed_lo] */
            {
                uint16_t speed = motor_hbridge_get_speed();
                response.data[0] = (uint8_t)(speed >> 8);
                response.data[1] = (uint8_t)(speed);
                response.data_len = 2;
            }
            break;

        case TSUMIKORO_CMD_DCMOTOR_GET_DIRECTION:
            /* Get current motor direction
             * Response: [direction] */
            response.data[0] = (uint8_t)motor_hbridge_get_direction();
            response.data_len = 1;
            break;

        case TSUMIKORO_CMD_DCMOTOR_ENABLE:
            /* Enable/disable motor output
             * Data: [enable (0=disable, 1=enable)] */
            if (packet->data_len >= 1) {
                bool enable = (packet->data[0] != 0);

                if (motor_hbridge_enable(enable)) {
                    response.data[0] = 0x00;  /* Success */
                } else {
                    response.data[0] = 0xFF;  /* Error */
                }
                response.data_len = 1;
            } else {
                response.data[0] = 0xFF;  /* Error - invalid length */
                response.data_len = 1;
            }
            break;

        case TSUMIKORO_CMD_DCMOTOR_ESTOP:
            /* Emergency stop (brake) */
            motor_hbridge_emergency_stop();
            response.data[0] = 0x00;  /* Success */
            response.data_len = 1;
            break;

        default:
            /* Unknown command - don't send response */
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
