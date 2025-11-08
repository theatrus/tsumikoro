/**
 * @file tsumikoro_tmc_hal_stm32.c
 * @brief STM32 HAL implementation for Tsumikoro TMC stepper driver
 */

#include "tsumikoro_tmc_hal_stm32.h"
#include <stddef.h>

/* Forward declarations of callback functions */
static void stm32_spi_begin(void);
static void stm32_spi_end(void);
static uint8_t stm32_spi_transfer(uint8_t data);
static void stm32_set_step_pin(bool state);
static void stm32_set_dir_pin(bool state);
static void stm32_set_enable_pin(bool state);
static void stm32_delay_us(uint32_t us);

/* Global pointer to current instance (for callbacks) */
static tsumikoro_tmc_hal_stm32_t *g_current_instance = NULL;

/* ============================================================================
 * STM32 HAL Callback Implementations
 * ============================================================================ */

/**
 * @brief SPI transaction begin (CS low)
 */
static void stm32_spi_begin(void)
{
    if (!g_current_instance) return;

    HAL_GPIO_WritePin(g_current_instance->hw_config.cs_port,
                     g_current_instance->hw_config.cs_pin,
                     GPIO_PIN_RESET);

    /* Small delay to ensure CS setup time */
    for (volatile int i = 0; i < 10; i++);
}

/**
 * @brief SPI transaction end (CS high)
 */
static void stm32_spi_end(void)
{
    if (!g_current_instance) return;

    /* Small delay before releasing CS */
    for (volatile int i = 0; i < 10; i++);

    HAL_GPIO_WritePin(g_current_instance->hw_config.cs_port,
                     g_current_instance->hw_config.cs_pin,
                     GPIO_PIN_SET);
}

/**
 * @brief SPI transfer single byte
 */
static uint8_t stm32_spi_transfer(uint8_t data)
{
    if (!g_current_instance || !g_current_instance->hw_config.spi_handle) {
        return 0;
    }

    uint8_t rx_data = 0;

    /* Blocking SPI transfer */
    HAL_SPI_TransmitReceive(g_current_instance->hw_config.spi_handle,
                           &data, &rx_data, 1, HAL_MAX_DELAY);

    return rx_data;
}

/**
 * @brief Set STEP pin state
 */
static void stm32_set_step_pin(bool state)
{
    if (!g_current_instance) return;

    if (g_current_instance->hw_config.step_port) {
        HAL_GPIO_WritePin(g_current_instance->hw_config.step_port,
                         g_current_instance->hw_config.step_pin,
                         state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Set DIR pin state
 */
static void stm32_set_dir_pin(bool state)
{
    if (!g_current_instance) return;

    if (g_current_instance->hw_config.dir_port) {
        HAL_GPIO_WritePin(g_current_instance->hw_config.dir_port,
                         g_current_instance->hw_config.dir_pin,
                         state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Set EN pin state (active low)
 */
static void stm32_set_enable_pin(bool state)
{
    if (!g_current_instance) return;

    if (g_current_instance->hw_config.en_port) {
        HAL_GPIO_WritePin(g_current_instance->hw_config.en_port,
                         g_current_instance->hw_config.en_pin,
                         state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Microsecond delay
 *
 * Crude delay implementation using busy-wait loop.
 * Assumes 64 MHz system clock (STM32G071).
 * For better accuracy, consider using hardware timer.
 */
static void stm32_delay_us(uint32_t us)
{
    /* At 64 MHz, approximately 64 cycles per microsecond
     * Adjust loop iterations based on compiler optimization */
    volatile uint32_t cycles = us * 16;  /* ~4 cycles per iteration */
    while (cycles--);
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

bool tsumikoro_tmc_hal_stm32_init(tsumikoro_tmc_hal_stm32_t *instance,
                                   const tsumikoro_tmc_hal_stm32_config_t *hw_config,
                                   tsumikoro_tmc_chip_type_t chip_type)
{
    if (!instance || !hw_config || !hw_config->spi_handle) {
        return false;
    }

    /* Store hardware configuration */
    instance->hw_config = *hw_config;

    /* Set global instance pointer for callbacks */
    g_current_instance = instance;

    /* Set up callback functions in TMC driver */
    instance->tmc.spi_begin = stm32_spi_begin;
    instance->tmc.spi_end = stm32_spi_end;
    instance->tmc.spi_transfer = stm32_spi_transfer;
    instance->tmc.set_step_pin = stm32_set_step_pin;
    instance->tmc.set_dir_pin = stm32_set_dir_pin;
    instance->tmc.set_enable_pin = stm32_set_enable_pin;
    instance->tmc.delay_us = stm32_delay_us;

    /* Initialize CS pin (idle high) */
    HAL_GPIO_WritePin(hw_config->cs_port, hw_config->cs_pin, GPIO_PIN_SET);

    /* Initialize STEP/DIR pins (low) */
    if (hw_config->step_port) {
        HAL_GPIO_WritePin(hw_config->step_port, hw_config->step_pin, GPIO_PIN_RESET);
    }
    if (hw_config->dir_port) {
        HAL_GPIO_WritePin(hw_config->dir_port, hw_config->dir_pin, GPIO_PIN_RESET);
    }

    /* Initialize EN pin (high = disabled) */
    if (hw_config->en_port) {
        HAL_GPIO_WritePin(hw_config->en_port, hw_config->en_pin, GPIO_PIN_SET);
    }

    /* Initialize TMC driver */
    if (!tsumikoro_tmc_init(&instance->tmc, chip_type)) {
        return false;
    }

    return true;
}

tsumikoro_tmc_stepper_t* tsumikoro_tmc_hal_stm32_get_driver(tsumikoro_tmc_hal_stm32_t *instance)
{
    if (!instance) {
        return NULL;
    }

    return &instance->tmc;
}
