/**
 * @file motor_hbridge.c
 * @brief H-Bridge motor driver implementation using STM32 LL drivers
 *
 * Hardware: DRV8876 in PH/EN mode (PMODE pin tied to VCC / left floating).
 * Pin mapping (see board schematic):
 *   - PA6: TIM16_CH1 PWM -> DRV EN (speed)
 *   - PA4: GPIO         -> DRV PH (direction)
 *
 * Note: In PH/EN mode, the DRV8876 does not support a true coast state.
 * Setting EN=0 activates synchronous low-side brake. MOTOR_DIR_COAST and
 * MOTOR_DIR_BRAKE therefore behave identically at the hardware level.
 * For full coast support, the DRV8876 nSLEEP line would need to be pulled
 * low, but that line is hard-tied to VCC on this board.
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
 * @brief Initialize TIM16 for PWM output (CH1)
 */
static void motor_init_timer(void)
{
    /* Enable TIM16 clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

    /* Disable timer before configuration */
    LL_TIM_DisableCounter(TIM16);

    /* Configure TIM16 for 20kHz PWM
     * Timer clock = 64 MHz (system clock)
     * Prescaler = 64 - 1 = 63 -> Timer frequency = 1 MHz
     * ARR = 50 - 1 = 49 -> PWM frequency = 20 kHz (50us period)
     */
    LL_TIM_SetPrescaler(TIM16, 64 - 1);                  /* 64 MHz / 64 = 1 MHz */
    LL_TIM_SetCounterMode(TIM16, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM16, MOTOR_PWM_PERIOD - 1);   /* 50 - 1 = 49 */
    LL_TIM_SetClockDivision(TIM16, LL_TIM_CLOCKDIVISION_DIV1);

    /* Disable ARR preload for immediate updates */
    LL_TIM_DisableARRPreload(TIM16);

    /* Reset counter */
    LL_TIM_SetCounter(TIM16, 0);

    /* Configure CH1 for PWM mode 1 */
    LL_TIM_OC_SetMode(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH1(TIM16, 0);  /* Start at 0% duty cycle */
    LL_TIM_OC_SetPolarity(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);

    /* Enable main output (required for TIM16/TIM17 - they have BDTR like advanced timers) */
    LL_TIM_EnableAllOutputs(TIM16);

    /* Enable timer */
    LL_TIM_EnableCounter(TIM16);

    /* Generate update event to load values */
    LL_TIM_GenerateEvent_UPDATE(TIM16);
}

/**
 * @brief Initialize GPIO pins for motor control
 */
static void motor_init_gpio(void)
{
    /* Enable GPIO clocks */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* PA6: TIM16_CH1 (PWM output -> DRV8876 EN) - AF5 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);  /* AF5 = TIM16 */
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);  /* High speed for 20kHz */
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);

    /* PA4: PH (direction control -> DRV8876 PH) - GPIO output */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);  /* Start with PH low (forward) */
}

/**
 * @brief Update direction control pin (PH) based on direction
 *
 * In PH/EN mode on the DRV8876:
 *   PH=0, EN=PWM -> forward
 *   PH=1, EN=PWM -> reverse
 *   EN=0         -> brake (synchronous low-side short)
 *
 * BRAKE and COAST both map to "zero out PWM" at the caller; this function
 * only sets the PH pin. The EN pin is controlled via the PWM duty cycle.
 */
static void motor_update_direction(motor_direction_t direction)
{
    switch (direction) {
        case MOTOR_DIR_FORWARD:
            /* PH = LOW -> forward */
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
            break;

        case MOTOR_DIR_REVERSE:
            /* PH = HIGH -> reverse */
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
            break;

        case MOTOR_DIR_BRAKE:
        case MOTOR_DIR_COAST:
        default:
            /* PH irrelevant - EN=0 (set at caller) causes synchronous brake */
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

    /* For BRAKE/COAST, force PWM to 0 (EN=0 -> DRV synchronous brake).
     * In PH/EN mode the DRV8876 cannot truly coast without pulling nSLEEP
     * low, which is hard-tied to VCC on this board. Both states are
     * therefore hardware-equivalent and documented as such.
     */
    uint16_t effective_speed = speed;
    if (direction == MOTOR_DIR_BRAKE || direction == MOTOR_DIR_COAST) {
        effective_speed = 0;
    }

    /* Update direction pin */
    motor_update_direction(direction);
    g_motor_state.current_direction = direction;

    /* Update PWM duty cycle
     * Speed range: 0-1000 (0-100%)
     * PWM period: MOTOR_PWM_PERIOD (50us @ 20kHz)
     * CCR = (speed * MOTOR_PWM_PERIOD) / 1000
     */
    uint16_t ccr_value = (effective_speed * MOTOR_PWM_PERIOD) / 1000;
    LL_TIM_OC_SetCompareCH1(TIM16, ccr_value);

    g_motor_state.current_speed = speed;
    g_motor_state.enabled = (speed > 0 && direction != MOTOR_DIR_COAST &&
                             direction != MOTOR_DIR_BRAKE);

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
