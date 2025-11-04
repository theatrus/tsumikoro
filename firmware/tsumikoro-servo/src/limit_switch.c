/**
 * @file limit_switch.c
 * @brief Limit switch input implementation using STM32 LL drivers
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "limit_switch.h"

bool limit_switch_init(void)
{
    /* Enable GPIO clock */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* Configure PA8 as input with pull-up
     * Pull-up is used for active-low switches (switch to GND)
     * This prevents floating input when switch is open */
    LL_GPIO_SetPinMode(LIMIT_SWITCH_GPIO_PORT, LIMIT_SWITCH_GPIO_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(LIMIT_SWITCH_GPIO_PORT, LIMIT_SWITCH_GPIO_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinSpeed(LIMIT_SWITCH_GPIO_PORT, LIMIT_SWITCH_GPIO_PIN, LL_GPIO_SPEED_FREQ_LOW);

    return true;
}

bool limit_switch_is_triggered(void)
{
    bool pin_state = LL_GPIO_IsInputPinSet(LIMIT_SWITCH_GPIO_PORT, LIMIT_SWITCH_GPIO_PIN);

#if LIMIT_SWITCH_ACTIVE_LOW
    /* Active low: switch triggered when pin is LOW (pulled to GND) */
    return !pin_state;
#else
    /* Active high: switch triggered when pin is HIGH (pulled to VDD) */
    return pin_state;
#endif
}

bool limit_switch_get_raw_state(void)
{
    return LL_GPIO_IsInputPinSet(LIMIT_SWITCH_GPIO_PORT, LIMIT_SWITCH_GPIO_PIN);
}
