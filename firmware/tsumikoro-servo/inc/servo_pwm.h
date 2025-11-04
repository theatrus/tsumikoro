/**
 * @file servo_pwm.h
 * @brief Servo PWM control using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_bus.h"

/* Configuration */
#define SERVO_PWM_FREQ_HZ           50      /**< Standard servo PWM frequency (50Hz = 20ms period) */
#define SERVO_PWM_TIMER_FREQ_HZ     1000000 /**< Timer frequency after prescaler (1 MHz) */
#define SERVO_PWM_PERIOD_US         20000   /**< PWM period in microseconds (20ms) */

#define SERVO_DEFAULT_MIN_PULSE_US  1000    /**< Default minimum pulse width (1ms = 0°) */
#define SERVO_DEFAULT_MAX_PULSE_US  2000    /**< Default maximum pulse width (2ms = 180°) */
#define SERVO_DEFAULT_CENTER_US     1500    /**< Default center position (1.5ms = 90°) */

#define SERVO_MAX_CHANNELS          4       /**< Maximum number of servo channels (PA6/PA7 reserved for ID) */

/* Position range: 0-1800 represents 0-180 degrees (tenths of degrees) */
#define SERVO_MIN_POSITION          0
#define SERVO_MAX_POSITION          1800

/**
 * @brief Servo channel configuration
 */
typedef struct {
    TIM_TypeDef *timer;         /**< Timer instance (TIM3, TIM14, TIM16, TIM17) */
    uint32_t channel;           /**< LL Timer channel (LL_TIM_CHANNEL_CHx) */
    GPIO_TypeDef *gpio_port;    /**< GPIO port for PWM output */
    uint32_t gpio_pin;          /**< GPIO pin for PWM output (LL_GPIO_PIN_x) */
    uint32_t gpio_af;           /**< GPIO alternate function (LL_GPIO_AF_x) */
    uint16_t min_pulse_us;      /**< Minimum pulse width in microseconds */
    uint16_t max_pulse_us;      /**< Maximum pulse width in microseconds */
    uint16_t current_position;  /**< Current position (0-1800) */
    uint16_t target_position;   /**< Target position (0-1800) */
    uint16_t speed;             /**< Movement speed (tenths of degrees per tick) */
    bool enabled;               /**< Channel enabled flag */
    bool moving;                /**< Currently moving flag */
} servo_channel_t;

/**
 * @brief Initialize servo PWM system
 *
 * Sets up timers and GPIO pins for servo control.
 * Configures all channels to default center position with output disabled.
 *
 * @return true if initialization successful, false otherwise
 */
bool servo_pwm_init(void);

/**
 * @brief Enable or disable a servo channel
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @param enable true to enable output, false to disable
 * @return true if successful, false if invalid channel
 */
bool servo_pwm_enable(uint8_t channel_index, bool enable);

/**
 * @brief Set target position for a servo channel
 *
 * Position will be reached gradually based on configured speed.
 * Position range: 0-1800 (0-180 degrees in tenths)
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @param position Target position (0-1800)
 * @return true if successful, false if invalid channel or position
 */
bool servo_pwm_set_position(uint8_t channel_index, uint16_t position);

/**
 * @brief Get current position of a servo channel
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @return Current position (0-1800), or 0 if invalid channel
 */
uint16_t servo_pwm_get_position(uint8_t channel_index);

/**
 * @brief Get target position of a servo channel
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @return Target position (0-1800), or 0 if invalid channel
 */
uint16_t servo_pwm_get_target_position(uint8_t channel_index);

/**
 * @brief Set movement speed for a servo channel
 *
 * Speed determines how fast the servo moves to target position.
 * Higher values = faster movement.
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @param speed Movement speed (tenths of degrees per process tick)
 * @return true if successful, false if invalid channel
 */
bool servo_pwm_set_speed(uint8_t channel_index, uint16_t speed);

/**
 * @brief Set pulse width calibration for a servo channel
 *
 * Allows customization of min/max pulse widths for different servo types.
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @param min_pulse_us Minimum pulse width in microseconds
 * @param max_pulse_us Maximum pulse width in microseconds
 * @return true if successful, false if invalid parameters
 */
bool servo_pwm_set_calibration(uint8_t channel_index, uint16_t min_pulse_us, uint16_t max_pulse_us);

/**
 * @brief Check if a servo is currently moving
 *
 * @param channel_index Channel index (0 to SERVO_MAX_CHANNELS-1)
 * @return true if servo is moving to target, false if at target or invalid channel
 */
bool servo_pwm_is_moving(uint8_t channel_index);

/**
 * @brief Process servo movement (call regularly, e.g. every 10ms)
 *
 * Updates servo positions toward their targets based on configured speeds.
 * Should be called from main loop or timer interrupt.
 */
void servo_pwm_process(void);

/**
 * @brief Get number of available servo channels
 *
 * @return Number of servo channels
 */
uint8_t servo_pwm_get_channel_count(void);

#endif /* SERVO_PWM_H */
