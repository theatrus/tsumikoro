/**
 * @file limit_switch.h
 * @brief Limit switch input using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_bus.h"

/* Configuration */
#define LIMIT_SWITCH_GPIO_PORT  GPIOA
#define LIMIT_SWITCH_GPIO_PIN   LL_GPIO_PIN_8
#define LIMIT_SWITCH_ACTIVE_LOW true  /**< true if switch pulls to GND when activated */

/**
 * @brief Initialize limit switch GPIO
 *
 * Configures PA8 as input with pull-up (for active-low switch).
 * Pin 16 on STM32G030F6P6 TSSOP20 package.
 *
 * @return true if initialization successful
 */
bool limit_switch_init(void);

/**
 * @brief Read limit switch state
 *
 * @return true if switch is activated (closed/triggered)
 *         false if switch is not activated (open/normal)
 */
bool limit_switch_is_triggered(void);

/**
 * @brief Get raw GPIO pin state
 *
 * @return true if pin is HIGH, false if pin is LOW
 */
bool limit_switch_get_raw_state(void);

#endif /* LIMIT_SWITCH_H */
