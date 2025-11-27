/**
 * @file tsumikoro_tmc_hal_stm32.h
 * @brief STM32 HAL implementation for Tsumikoro TMC stepper driver
 *
 * Provides STM32-specific SPI and GPIO callbacks for TMC2130/TMC5160 drivers.
 * Designed for STM32G0 series but should work with other STM32 families.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_TMC_HAL_STM32_H
#define TSUMIKORO_TMC_HAL_STM32_H

#include "tsumikoro_tmc_stepper.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_spi.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief STM32-specific TMC hardware configuration
 */
typedef struct {
    /* SPI peripheral */
    SPI_HandleTypeDef *spi_handle;      /**< Pointer to HAL SPI handle */

    /* GPIO pins */
    GPIO_TypeDef *cs_port;              /**< Chip Select port */
    uint16_t cs_pin;                    /**< Chip Select pin */

    GPIO_TypeDef *step_port;            /**< Step port (TMC2130 only) */
    uint16_t step_pin;                  /**< Step pin (TMC2130 only) */

    GPIO_TypeDef *dir_port;             /**< Direction port (TMC2130 only) */
    uint16_t dir_pin;                   /**< Direction pin (TMC2130 only) */

    GPIO_TypeDef *en_port;              /**< Enable port */
    uint16_t en_pin;                    /**< Enable pin */
} tsumikoro_tmc_hal_stm32_config_t;

/**
 * @brief STM32 TMC driver instance
 *
 * Contains both the generic TMC driver and STM32-specific hardware config.
 */
typedef struct {
    tsumikoro_tmc_stepper_t tmc;        /**< Generic TMC driver */
    tsumikoro_tmc_hal_stm32_config_t hw_config;  /**< STM32 hardware config */
} tsumikoro_tmc_hal_stm32_t;

/**
 * @brief Initialize TMC driver with STM32 hardware
 *
 * Sets up the TMC driver with STM32-specific SPI and GPIO callbacks.
 * Note: SPI peripheral must be initialized before calling this function.
 *
 * @param instance Pointer to STM32 TMC instance
 * @param hw_config STM32 hardware configuration
 * @param chip_type TMC chip type (TMC2130 or TMC5160)
 * @return true on success, false on error
 */
bool tsumikoro_tmc_hal_stm32_init(tsumikoro_tmc_hal_stm32_t *instance,
                                   const tsumikoro_tmc_hal_stm32_config_t *hw_config,
                                   tsumikoro_tmc_chip_type_t chip_type);

/**
 * @brief Get the generic TMC driver handle
 *
 * @param instance Pointer to STM32 TMC instance
 * @return Pointer to generic TMC driver
 */
tsumikoro_tmc_stepper_t* tsumikoro_tmc_hal_stm32_get_driver(tsumikoro_tmc_hal_stm32_t *instance);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_TMC_HAL_STM32_H */
