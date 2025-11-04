/**
 * @file servo_pwm.c
 * @brief Servo PWM control implementation using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "servo_pwm.h"
#include <string.h>

/* Private variables */
static servo_channel_t g_servo_channels[SERVO_MAX_CHANNELS];

/**
 * @brief Initialize a single timer for PWM output
 *
 * @param timer Timer instance to configure
 */
static void servo_pwm_init_timer(TIM_TypeDef *timer)
{
    /* Enable timer clock */
    if (timer == TIM3) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    } else if (timer == TIM14) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM14);
    } else if (timer == TIM16) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
    } else if (timer == TIM17) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
    }

    /* Disable timer before configuration */
    LL_TIM_DisableCounter(timer);

    /* Configure timer for 50Hz PWM
     * Timer clock = 64 MHz (system clock)
     * Prescaler = 64 - 1 = 63 → Timer frequency = 1 MHz
     * ARR = 20000 - 1 = 19999 → PWM frequency = 50 Hz (20ms period)
     */
    LL_TIM_SetPrescaler(timer, 64 - 1);                    /* 64 MHz / 64 = 1 MHz */
    LL_TIM_SetCounterMode(timer, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(timer, SERVO_PWM_PERIOD_US - 1);  /* 20000 - 1 = 19999 */
    LL_TIM_SetClockDivision(timer, LL_TIM_CLOCKDIVISION_DIV1);

    /* Disable ARR preload for immediate updates */
    LL_TIM_DisableARRPreload(timer);

    /* Reset counter */
    LL_TIM_SetCounter(timer, 0);
}

/**
 * @brief Configure a timer channel for PWM output
 *
 * @param timer Timer instance
 * @param channel LL Timer channel
 */
static void servo_pwm_init_channel(TIM_TypeDef *timer, uint32_t channel)
{
    /* Configure PWM mode 1 (active when CNT < CCR) */
    LL_TIM_OC_SetMode(timer, channel, LL_TIM_OCMODE_PWM1);

    /* Set initial compare value (center position) */
    switch (channel) {
        case LL_TIM_CHANNEL_CH1:
            LL_TIM_OC_SetCompareCH1(timer, SERVO_DEFAULT_CENTER_US);
            LL_TIM_OC_SetPolarity(timer, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
            LL_TIM_CC_EnableChannel(timer, LL_TIM_CHANNEL_CH1);
            break;
        case LL_TIM_CHANNEL_CH2:
            LL_TIM_OC_SetCompareCH2(timer, SERVO_DEFAULT_CENTER_US);
            LL_TIM_OC_SetPolarity(timer, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
            LL_TIM_CC_EnableChannel(timer, LL_TIM_CHANNEL_CH2);
            break;
        case LL_TIM_CHANNEL_CH3:
            LL_TIM_OC_SetCompareCH3(timer, SERVO_DEFAULT_CENTER_US);
            LL_TIM_OC_SetPolarity(timer, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
            LL_TIM_CC_EnableChannel(timer, LL_TIM_CHANNEL_CH3);
            break;
        case LL_TIM_CHANNEL_CH4:
            LL_TIM_OC_SetCompareCH4(timer, SERVO_DEFAULT_CENTER_US);
            LL_TIM_OC_SetPolarity(timer, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
            LL_TIM_CC_EnableChannel(timer, LL_TIM_CHANNEL_CH4);
            break;
    }

    /* Disable fast mode */
    LL_TIM_OC_DisableFast(timer, channel);

    /* Enable preload for smooth transitions */
    LL_TIM_OC_EnablePreload(timer, channel);
}

/**
 * @brief Configure GPIO pin for timer alternate function
 *
 * @param gpio_port GPIO port
 * @param gpio_pin GPIO pin (LL_GPIO_PIN_x)
 * @param gpio_af Alternate function (LL_GPIO_AF_x)
 */
static void servo_pwm_init_gpio(GPIO_TypeDef *gpio_port, uint32_t gpio_pin, uint32_t gpio_af)
{
    /* Enable GPIO clock */
    if (gpio_port == GPIOA) {
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    } else if (gpio_port == GPIOB) {
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    }

    /* Get pin position (0-15) from LL pin value */
    uint32_t pin_pos = __builtin_ctz(gpio_pin);

    /* Configure pin mode as alternate function */
    LL_GPIO_SetPinMode(gpio_port, gpio_pin, LL_GPIO_MODE_ALTERNATE);

    /* Set alternate function */
    if (pin_pos < 8) {
        LL_GPIO_SetAFPin_0_7(gpio_port, gpio_pin, gpio_af);
    } else {
        LL_GPIO_SetAFPin_8_15(gpio_port, gpio_pin, gpio_af);
    }

    /* Configure output type as push-pull */
    LL_GPIO_SetPinOutputType(gpio_port, gpio_pin, LL_GPIO_OUTPUT_PUSHPULL);

    /* Set speed (low is fine for 50Hz PWM) */
    LL_GPIO_SetPinSpeed(gpio_port, gpio_pin, LL_GPIO_SPEED_FREQ_LOW);

    /* No pull-up/pull-down */
    LL_GPIO_SetPinPull(gpio_port, gpio_pin, LL_GPIO_PULL_NO);
}

/**
 * @brief Convert position (0-1800) to pulse width in microseconds
 *
 * @param channel Servo channel
 * @return Pulse width in microseconds
 */
static uint16_t servo_pwm_position_to_pulse(const servo_channel_t *channel)
{
    /* Linear interpolation: position -> pulse width
     * position = 0    → min_pulse_us
     * position = 1800 → max_pulse_us
     */
    uint32_t range = channel->max_pulse_us - channel->min_pulse_us;
    uint32_t pulse_us = channel->min_pulse_us +
                        ((channel->current_position * range) / SERVO_MAX_POSITION);

    /* Clamp to valid range */
    if (pulse_us < channel->min_pulse_us) {
        pulse_us = channel->min_pulse_us;
    }
    if (pulse_us > channel->max_pulse_us) {
        pulse_us = channel->max_pulse_us;
    }

    return (uint16_t)pulse_us;
}

/**
 * @brief Update PWM duty cycle for a channel
 *
 * @param channel Servo channel
 */
static void servo_pwm_update_channel(servo_channel_t *channel)
{
    if (!channel->enabled) {
        return;
    }

    uint16_t pulse_us = servo_pwm_position_to_pulse(channel);

    /* Update compare value (CCR) - this is in microseconds at 1MHz timer clock */
    LL_TIM_OC_SetCompareCH1(channel->timer, pulse_us);

    /* Handle different channels */
    switch (channel->channel) {
        case LL_TIM_CHANNEL_CH1:
            LL_TIM_OC_SetCompareCH1(channel->timer, pulse_us);
            break;
        case LL_TIM_CHANNEL_CH2:
            LL_TIM_OC_SetCompareCH2(channel->timer, pulse_us);
            break;
        case LL_TIM_CHANNEL_CH3:
            LL_TIM_OC_SetCompareCH3(channel->timer, pulse_us);
            break;
        case LL_TIM_CHANNEL_CH4:
            LL_TIM_OC_SetCompareCH4(channel->timer, pulse_us);
            break;
    }
}

bool servo_pwm_init(void)
{
    /* Initialize servo channels with configuration
     * Pin mapping (STM32G030F6P6 TSSOP20):
     * Channel 0: PA0 - TIM3_CH1 (AF1)
     * Channel 1: PA2 - TIM3_CH3 (AF1)
     * Channel 2: PA3 - TIM3_CH4 (AF1)
     * Channel 3: PA4 - TIM14_CH1 (AF4)
     *
     * Reserved pins:
     * - PA6, PA7: Hardware ID generation (inputs with pull-up/down)
     * - PA13, PA14: SWD (SWDIO, SWCLK) - DO NOT USE
     * - PA9, PA10: USART1 (bus communication)
     * - PA1: RS-485 DE
     * - PA5: Status LED
     */

    memset(g_servo_channels, 0, sizeof(g_servo_channels));

    /* Channel 0: PA0 - TIM3_CH1 (AF1) */
    g_servo_channels[0].timer = TIM3;
    g_servo_channels[0].channel = LL_TIM_CHANNEL_CH1;
    g_servo_channels[0].gpio_port = GPIOA;
    g_servo_channels[0].gpio_pin = LL_GPIO_PIN_0;
    g_servo_channels[0].gpio_af = LL_GPIO_AF_1;
    g_servo_channels[0].min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    g_servo_channels[0].max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    g_servo_channels[0].current_position = 900;  /* 90 degrees */
    g_servo_channels[0].target_position = 900;
    g_servo_channels[0].speed = 10;  /* 1 degree per tick */
    g_servo_channels[0].enabled = false;

    /* Channel 1: PA2 - TIM3_CH3 (AF1) */
    g_servo_channels[1].timer = TIM3;
    g_servo_channels[1].channel = LL_TIM_CHANNEL_CH3;
    g_servo_channels[1].gpio_port = GPIOA;
    g_servo_channels[1].gpio_pin = LL_GPIO_PIN_2;
    g_servo_channels[1].gpio_af = LL_GPIO_AF_1;
    g_servo_channels[1].min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    g_servo_channels[1].max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    g_servo_channels[1].current_position = 900;
    g_servo_channels[1].target_position = 900;
    g_servo_channels[1].speed = 10;
    g_servo_channels[1].enabled = false;

    /* Channel 2: PA3 - TIM3_CH4 (AF1) */
    g_servo_channels[2].timer = TIM3;
    g_servo_channels[2].channel = LL_TIM_CHANNEL_CH4;
    g_servo_channels[2].gpio_port = GPIOA;
    g_servo_channels[2].gpio_pin = LL_GPIO_PIN_3;
    g_servo_channels[2].gpio_af = LL_GPIO_AF_1;
    g_servo_channels[2].min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    g_servo_channels[2].max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    g_servo_channels[2].current_position = 900;
    g_servo_channels[2].target_position = 900;
    g_servo_channels[2].speed = 10;
    g_servo_channels[2].enabled = false;

    /* Channel 3: PA4 - TIM14_CH1 (AF4) */
    g_servo_channels[3].timer = TIM14;
    g_servo_channels[3].channel = LL_TIM_CHANNEL_CH1;
    g_servo_channels[3].gpio_port = GPIOA;
    g_servo_channels[3].gpio_pin = LL_GPIO_PIN_4;
    g_servo_channels[3].gpio_af = LL_GPIO_AF_4;
    g_servo_channels[3].min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    g_servo_channels[3].max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    g_servo_channels[3].current_position = 900;
    g_servo_channels[3].target_position = 900;
    g_servo_channels[3].speed = 10;
    g_servo_channels[3].enabled = false;

    /* Initialize timers (unique timers only) */
    servo_pwm_init_timer(TIM3);   /* Used by channels 0, 1, 2 */
    servo_pwm_init_timer(TIM14);  /* Used by channel 3 */

    /* Initialize channels and GPIO pins */
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        servo_pwm_init_channel(g_servo_channels[i].timer, g_servo_channels[i].channel);
        servo_pwm_init_gpio(g_servo_channels[i].gpio_port,
                           g_servo_channels[i].gpio_pin,
                           g_servo_channels[i].gpio_af);
    }

    /* Enable timers */
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableCounter(TIM14);

    /* Generate update event to load values */
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM14);

    return true;
}

bool servo_pwm_enable(uint8_t channel_index, bool enable)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return false;
    }

    g_servo_channels[channel_index].enabled = enable;

    if (enable) {
        /* Update PWM output immediately */
        servo_pwm_update_channel(&g_servo_channels[channel_index]);
    } else {
        /* Optionally disable output or set to safe position */
        /* For now, just mark as disabled - PWM continues but won't update */
    }

    return true;
}

bool servo_pwm_set_position(uint8_t channel_index, uint16_t position)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return false;
    }

    if (position > SERVO_MAX_POSITION) {
        position = SERVO_MAX_POSITION;
    }

    g_servo_channels[channel_index].target_position = position;
    g_servo_channels[channel_index].moving =
        (g_servo_channels[channel_index].current_position != position);

    return true;
}

uint16_t servo_pwm_get_position(uint8_t channel_index)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return 0;
    }

    return g_servo_channels[channel_index].current_position;
}

uint16_t servo_pwm_get_target_position(uint8_t channel_index)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return 0;
    }

    return g_servo_channels[channel_index].target_position;
}

bool servo_pwm_set_speed(uint8_t channel_index, uint16_t speed)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return false;
    }

    g_servo_channels[channel_index].speed = speed;
    return true;
}

bool servo_pwm_set_calibration(uint8_t channel_index, uint16_t min_pulse_us, uint16_t max_pulse_us)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return false;
    }

    /* Validate pulse widths */
    if (min_pulse_us < 500 || min_pulse_us > 2500) {
        return false;
    }
    if (max_pulse_us < 500 || max_pulse_us > 2500) {
        return false;
    }
    if (min_pulse_us >= max_pulse_us) {
        return false;
    }

    g_servo_channels[channel_index].min_pulse_us = min_pulse_us;
    g_servo_channels[channel_index].max_pulse_us = max_pulse_us;

    /* Update PWM immediately to reflect new calibration */
    servo_pwm_update_channel(&g_servo_channels[channel_index]);

    return true;
}

bool servo_pwm_is_moving(uint8_t channel_index)
{
    if (channel_index >= SERVO_MAX_CHANNELS) {
        return false;
    }

    return g_servo_channels[channel_index].moving;
}

void servo_pwm_process(void)
{
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        servo_channel_t *ch = &g_servo_channels[i];

        if (!ch->enabled || !ch->moving) {
            continue;
        }

        /* Move toward target position */
        if (ch->current_position < ch->target_position) {
            /* Moving toward higher position */
            uint16_t step = ch->speed;
            if (ch->current_position + step > ch->target_position) {
                ch->current_position = ch->target_position;
                ch->moving = false;
            } else {
                ch->current_position += step;
            }
        } else if (ch->current_position > ch->target_position) {
            /* Moving toward lower position */
            uint16_t step = ch->speed;
            if (ch->current_position < step + ch->target_position) {
                ch->current_position = ch->target_position;
                ch->moving = false;
            } else {
                ch->current_position -= step;
            }
        } else {
            /* At target */
            ch->moving = false;
        }

        /* Update PWM output */
        servo_pwm_update_channel(ch);
    }
}

uint8_t servo_pwm_get_channel_count(void)
{
    return SERVO_MAX_CHANNELS;
}
