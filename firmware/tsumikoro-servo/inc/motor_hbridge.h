/**
 * @file motor_hbridge.h
 * @brief H-Bridge motor driver control using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_HBRIDGE_H
#define MOTOR_HBRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_bus.h"

/* Configuration */
#define MOTOR_PWM_FREQ_HZ      20000    /**< Motor PWM frequency (20kHz - above audible range) */
#define MOTOR_TIMER_FREQ_HZ    1000000  /**< Timer frequency after prescaler (1 MHz) */
#define MOTOR_PWM_PERIOD       50       /**< PWM period in microseconds (20kHz) */

/**
 * @brief Motor direction enumeration
 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,   /**< Forward direction */
    MOTOR_DIR_REVERSE = 1,   /**< Reverse direction */
    MOTOR_DIR_BRAKE   = 2,   /**< Active brake (both high or both low) */
    MOTOR_DIR_COAST   = 3    /**< Coast/free-running (both low, PWM disabled) */
} motor_direction_t;

/**
 * @brief Initialize H-bridge motor driver
 *
 * Sets up TIM1_CH4 for PWM speed control and GPIO pins for direction.
 * Pin mapping:
 * - PA11: TIM1_CH4 (PWM speed control)
 * - PA12: IN1 (direction control)
 * - PB9: IN2 (direction control)
 *
 * @return true if initialization successful, false otherwise
 */
bool motor_hbridge_init(void);

/**
 * @brief Set motor speed and direction
 *
 * @param speed Motor speed (0-1000, where 1000 = 100%)
 * @param direction Motor direction (FORWARD, REVERSE, BRAKE, COAST)
 * @return true if successful, false if invalid parameters
 */
bool motor_hbridge_set(uint16_t speed, motor_direction_t direction);

/**
 * @brief Get current motor speed
 *
 * @return Current speed (0-1000)
 */
uint16_t motor_hbridge_get_speed(void);

/**
 * @brief Get current motor direction
 *
 * @return Current direction
 */
motor_direction_t motor_hbridge_get_direction(void);

/**
 * @brief Emergency stop (active brake)
 *
 * Immediately sets motor to brake mode with 0 speed.
 */
void motor_hbridge_emergency_stop(void);

/**
 * @brief Enable or disable motor output
 *
 * @param enable true to enable motor, false to coast
 * @return true if successful
 */
bool motor_hbridge_enable(bool enable);

#endif /* MOTOR_HBRIDGE_H */
