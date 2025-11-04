/**
 * @file motor_hbridge.c
 * @brief H-Bridge motor driver implementation using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_hbridge.h"

/* Private variables */
static struct {
    uint16_t current_speed;             /**< Current speed (0-1000) */
    motor_direction_t current_direction; /**< Current direction */
    bool enabled;                        /**< Motor output enabled */
} g_motor_state = {
    .current_speed = 0,
    .current_direction = MOTOR_DIR_COAST,
    .enabled = false
};

/**
 * @brief Initialize TIM1 for PWM output
 */
static void motor_init_timer(void)
{
    /* Enable TIM1 clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Disable timer before configuration */
    LL_TIM_DisableCounter(TIM1);

    /* Configure TIM1 for 20kHz PWM
     * Timer clock = 64 MHz (system clock)
     * Prescaler = 64 - 1 = 63 → Timer frequency = 1 MHz
     * ARR = 50 - 1 = 49 → PWM frequency = 20 kHz (50µs period)
     */
    LL_TIM_SetPrescaler(TIM1, 64 - 1);                  /* 64 MHz / 64 = 1 MHz */
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM1, MOTOR_PWM_PERIOD - 1);   /* 50 - 1 = 49 */
    LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);

    /* Disable ARR preload for immediate updates */
    LL_TIM_DisableARRPreload(TIM1);

    /* Reset counter */
    LL_TIM_SetCounter(TIM1, 0);

    /* Configure CH4 for PWM mode 1 */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH4(TIM1, 0);  /* Start at 0% duty cycle */
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

    /* Enable main output (required for advanced timers like TIM1) */
    LL_TIM_EnableAllOutputs(TIM1);

    /* Enable timer */
    LL_TIM_EnableCounter(TIM1);

    /* Generate update event to load values */
    LL_TIM_GenerateEvent_UPDATE(TIM1);
}

/**
 * @brief Initialize GPIO pins for motor control
 */
static void motor_init_gpio(void)
{
    /* Enable GPIO clocks */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /* PA11: TIM1_CH4 (PWM output) */
    uint32_t pa11_pin_pos = __builtin_ctz(LL_GPIO_PIN_11);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    if (pa11_pin_pos < 8) {
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_2);  /* AF2 = TIM1 */
    } else {
        LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_2);
    }
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);  /* High speed for 20kHz */
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);

    /* PA12: IN1 (direction control - GPIO output) */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);  /* Start low */

    /* PB9: IN2 (direction control - GPIO output) */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);  /* Start low */
}

/**
 * @brief Update direction control pins based on direction
 */
static void motor_update_direction(motor_direction_t direction)
{
    switch (direction) {
        case MOTOR_DIR_FORWARD:
            /* IN1 = HIGH, IN2 = LOW */
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12);
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
            break;

        case MOTOR_DIR_REVERSE:
            /* IN1 = LOW, IN2 = HIGH */
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
            break;

        case MOTOR_DIR_BRAKE:
            /* IN1 = HIGH, IN2 = HIGH (or both LOW for some drivers) */
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12);
            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
            break;

        case MOTOR_DIR_COAST:
        default:
            /* IN1 = LOW, IN2 = LOW */
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
            break;
    }
}

bool motor_hbridge_init(void)
{
    /* Initialize state */
    g_motor_state.current_speed = 0;
    g_motor_state.current_direction = MOTOR_DIR_COAST;
    g_motor_state.enabled = false;

    /* Initialize hardware */
    motor_init_timer();
    motor_init_gpio();

    /* Set initial direction (coast) */
    motor_update_direction(MOTOR_DIR_COAST);

    return true;
}

bool motor_hbridge_set(uint16_t speed, motor_direction_t direction)
{
    /* Validate speed (0-1000 = 0-100%) */
    if (speed > 1000) {
        speed = 1000;
    }

    /* Validate direction */
    if (direction > MOTOR_DIR_COAST) {
        return false;
    }

    /* Update direction pins */
    motor_update_direction(direction);
    g_motor_state.current_direction = direction;

    /* Update PWM duty cycle
     * Speed range: 0-1000 (0-100%)
     * PWM period: MOTOR_PWM_PERIOD (50µs @ 20kHz)
     * CCR = (speed * MOTOR_PWM_PERIOD) / 1000
     */
    uint16_t ccr_value = (speed * MOTOR_PWM_PERIOD) / 1000;
    LL_TIM_OC_SetCompareCH4(TIM1, ccr_value);

    g_motor_state.current_speed = speed;
    g_motor_state.enabled = (speed > 0 && direction != MOTOR_DIR_COAST);

    return true;
}

uint16_t motor_hbridge_get_speed(void)
{
    return g_motor_state.current_speed;
}

motor_direction_t motor_hbridge_get_direction(void)
{
    return g_motor_state.current_direction;
}

void motor_hbridge_emergency_stop(void)
{
    /* Set brake mode with 0 speed */
    motor_hbridge_set(0, MOTOR_DIR_BRAKE);
}

bool motor_hbridge_enable(bool enable)
{
    if (enable) {
        /* Keep current settings */
        g_motor_state.enabled = true;
    } else {
        /* Coast mode (disable output) */
        motor_hbridge_set(0, MOTOR_DIR_COAST);
        g_motor_state.enabled = false;
    }

    return true;
}
