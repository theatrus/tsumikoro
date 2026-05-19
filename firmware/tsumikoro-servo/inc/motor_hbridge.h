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
    MOTOR_DIR_BRAKE   = 2,   /**< Synchronous brake (DRV EN=0) */
    MOTOR_DIR_COAST   = 3    /**< Coast request. NOTE: hardware-equivalent
                                  to BRAKE on this board because nSLEEP is
                                  hard-tied high. See motor_hbridge.c. */
} motor_direction_t;

/**
 * @brief Initialize H-bridge motor driver
 *
 * Sets up TIM16_CH1 for PWM speed control and a GPIO for direction.
 * Hardware: DRV8876 in PH/EN mode (PMODE tied to VCC on the board).
 *
 * Pin mapping (STM32G030F6P6 TSSOP-20):
 * - PA6: TIM16_CH1 (AF5) -> DRV8876 EN (pin 1)  - PWM speed
 * - PA4: GPIO            -> DRV8876 PH (pin 2)  - direction
 *
 * Earlier revisions used PA11/PA12/PB9 with TIM1_CH4. Those pins now host
 * USART1 (PA9/PA10 via SYSCFG remap) for RS-485 comms, and I2C1 (PB8/PB9)
 * for external peripherals. See CLAUDE.md for the full pin map.
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
