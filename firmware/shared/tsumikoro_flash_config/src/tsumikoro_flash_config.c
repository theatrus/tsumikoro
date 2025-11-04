/**
 * @file tsumikoro_flash_config.c
 * @brief Flash configuration storage implementation using direct register access
 *
 * Uses LL-style direct register manipulation for flash operations.
 * No HAL dependencies.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_flash_config.h"
#include <string.h>

/* Platform-specific includes - must be provided by build system */
#if defined(STM32G030xx)
/* Include device-specific header directly (no HAL dependency) */
#include "stm32g030xx.h"
#elif defined(STM32G0)
/* Generic STM32G0 header */
#include "stm32g0xx.h"
#elif defined(ESP32)
#error "ESP32 flash config not yet implemented"
#else
#error "Unsupported platform for flash config"
#endif

/**
 * @brief Flash unlock keys
 */
#define FLASH_KEY1              0x45670123UL
#define FLASH_KEY2              0xCDEF89ABUL

/**
 * @brief Flash page size (2KB for STM32G0)
 */
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         0x800UL  /* 2048 bytes */
#endif

/**
 * @brief Flash configuration location
 *
 * STM32G030F6P6: 32KB flash = 16 pages of 2KB each
 * Config stored in last page (Page 15) at 0x08007800
 */
#if defined(STM32G030xx)
#define FLASH_CONFIG_PAGE       15
#define FLASH_CONFIG_ADDR       (FLASH_BASE + (FLASH_CONFIG_PAGE * FLASH_PAGE_SIZE))
#define FLASH_PAGE_SIZE_BYTES   FLASH_PAGE_SIZE
#else
#error "Flash config address not defined for this device"
#endif

/**
 * @brief Timeout for flash operations (in loop iterations)
 */
#define FLASH_TIMEOUT           50000UL

/**
 * @brief CRC32 polynomial (standard Ethernet CRC32)
 */
#define CRC32_POLYNOMIAL        0xEDB88320UL

/**
 * @brief Calculate CRC32 using software implementation
 *
 * Uses the standard Ethernet CRC32 polynomial.
 *
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return Calculated CRC32 value
 */
static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

/**
 * @brief Calculate CRC32 for configuration structure
 *
 * @param config Pointer to configuration structure
 * @return Calculated CRC32 (excludes the crc32 field itself)
 */
static uint32_t config_calculate_crc(const tsumikoro_config_t *config)
{
    /* CRC covers everything except the crc32 field at the end */
    size_t crc_length = sizeof(tsumikoro_config_t) - sizeof(uint32_t);
    return calculate_crc32((const uint8_t *)config, crc_length);
}

/**
 * @brief Validate configuration structure
 *
 * @param config Pointer to configuration structure
 * @return true if valid (magic and CRC match), false otherwise
 */
static bool config_validate(const tsumikoro_config_t *config)
{
    if (config->magic != TSUMIKORO_CONFIG_MAGIC) {
        return false;
    }

    uint32_t calculated_crc = config_calculate_crc(config);
    return (calculated_crc == config->crc32);
}

/**
 * @brief Unlock flash for programming (LL-style)
 *
 * @return true if successfully unlocked, false if already unlocked or error
 */
static bool flash_unlock(void)
{
    /* Check if already unlocked */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
        return true;  /* Already unlocked */
    }

    /* Write unlock sequence */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* Verify unlock */
    return ((FLASH->CR & FLASH_CR_LOCK) == 0);
}

/**
 * @brief Lock flash after programming (LL-style)
 */
static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief Wait for flash operation to complete (LL-style)
 *
 * @return TSUMIKORO_FLASH_OK on success, error code on timeout or error
 */
static tsumikoro_flash_status_t flash_wait_for_last_operation(void)
{
    uint32_t timeout = FLASH_TIMEOUT;

    /* Wait for BSY flag to clear */
    while ((FLASH->SR & FLASH_SR_BSY1) && (timeout > 0)) {
        timeout--;
    }

    if (timeout == 0) {
        return TSUMIKORO_FLASH_ERROR_WRITE;
    }

    /* Check for errors */
    uint32_t error_flags = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
                           FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR |
                           FLASH_SR_MISERR | FLASH_SR_FASTERR;

    if (FLASH->SR & error_flags) {
        /* Clear error flags */
        FLASH->SR = error_flags;
        return TSUMIKORO_FLASH_ERROR_WRITE;
    }

    /* Clear EOP flag if set */
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;
    }

    return TSUMIKORO_FLASH_OK;
}

/**
 * @brief Erase flash page (LL-style)
 *
 * @param page Page number to erase
 * @return TSUMIKORO_FLASH_OK on success, error code otherwise
 */
static tsumikoro_flash_status_t flash_erase_page(uint32_t page)
{
    /* Wait for any ongoing operation */
    tsumikoro_flash_status_t status = flash_wait_for_last_operation();
    if (status != TSUMIKORO_FLASH_OK) {
        return status;
    }

    /* Set page erase bit and page number */
    FLASH->CR &= ~FLASH_CR_PNB;  /* Clear page number field */
    FLASH->CR |= FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);

    /* Start erase */
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for completion */
    status = flash_wait_for_last_operation();

    /* Clear PER bit */
    FLASH->CR &= ~FLASH_CR_PER;

    return status;
}

/**
 * @brief Program doubleword (64-bit) to flash (LL-style)
 *
 * @param address Flash address (must be 64-bit aligned)
 * @param data 64-bit data to write
 * @return TSUMIKORO_FLASH_OK on success, error code otherwise
 */
static tsumikoro_flash_status_t flash_program_doubleword(uint32_t address, uint64_t data)
{
    /* Wait for any ongoing operation */
    tsumikoro_flash_status_t status = flash_wait_for_last_operation();
    if (status != TSUMIKORO_FLASH_OK) {
        return status;
    }

    /* Set PG bit to enable programming */
    FLASH->CR |= FLASH_CR_PG;

    /* Write first word */
    *(__IO uint32_t *)address = (uint32_t)data;
    /* Write second word (triggers actual programming) */
    *(__IO uint32_t *)(address + 4) = (uint32_t)(data >> 32);

    /* Wait for completion */
    status = flash_wait_for_last_operation();

    /* Clear PG bit */
    FLASH->CR &= ~FLASH_CR_PG;

    return status;
}

tsumikoro_flash_status_t tsumikoro_flash_config_init(void)
{
    /* No initialization needed for direct register access */
    return TSUMIKORO_FLASH_OK;
}

tsumikoro_flash_status_t tsumikoro_flash_config_load(tsumikoro_config_t *config)
{
    if (config == NULL) {
        return TSUMIKORO_FLASH_ERROR_INVALID;
    }

    /* Read configuration from flash */
    const tsumikoro_config_t *flash_config = (const tsumikoro_config_t *)FLASH_CONFIG_ADDR;

    /* Validate before copying */
    if (!config_validate(flash_config)) {
        return TSUMIKORO_FLASH_ERROR_INVALID;
    }

    /* Copy to user buffer */
    memcpy(config, flash_config, sizeof(tsumikoro_config_t));

    return TSUMIKORO_FLASH_OK;
}

tsumikoro_flash_status_t tsumikoro_flash_config_save(const tsumikoro_config_t *config)
{
    if (config == NULL) {
        return TSUMIKORO_FLASH_ERROR_INVALID;
    }

    /* Create a copy with updated CRC */
    tsumikoro_config_t config_copy;
    memcpy(&config_copy, config, sizeof(tsumikoro_config_t));
    config_copy.magic = TSUMIKORO_CONFIG_MAGIC;
    config_copy.version = TSUMIKORO_CONFIG_VERSION;
    config_copy.crc32 = config_calculate_crc(&config_copy);

    /* Unlock flash */
    if (!flash_unlock()) {
        return TSUMIKORO_FLASH_ERROR_LOCKED;
    }

    /* Erase flash page */
    tsumikoro_flash_status_t status = flash_erase_page(FLASH_CONFIG_PAGE);
    if (status != TSUMIKORO_FLASH_OK) {
        flash_lock();
        return status;
    }

    /* Write configuration to flash (64-bit writes required by STM32G0) */
    uint64_t *src = (uint64_t *)&config_copy;
    uint32_t dest_addr = FLASH_CONFIG_ADDR;
    size_t words_to_write = (sizeof(tsumikoro_config_t) + 7) / 8;  /* Round up to 64-bit words */

    for (size_t i = 0; i < words_to_write; i++) {
        status = flash_program_doubleword(dest_addr, src[i]);
        if (status != TSUMIKORO_FLASH_OK) {
            flash_lock();
            return TSUMIKORO_FLASH_ERROR_WRITE;
        }
        dest_addr += 8;
    }

    /* Lock flash */
    flash_lock();

    /* Verify write */
    const tsumikoro_config_t *flash_config = (const tsumikoro_config_t *)FLASH_CONFIG_ADDR;
    if (!config_validate(flash_config)) {
        return TSUMIKORO_FLASH_ERROR_VERIFY;
    }

    return TSUMIKORO_FLASH_OK;
}

tsumikoro_flash_status_t tsumikoro_flash_config_erase(void)
{
    /* Unlock flash */
    if (!flash_unlock()) {
        return TSUMIKORO_FLASH_ERROR_LOCKED;
    }

    /* Erase flash page */
    tsumikoro_flash_status_t status = flash_erase_page(FLASH_CONFIG_PAGE);

    /* Lock flash */
    flash_lock();

    return status;
}

bool tsumikoro_flash_config_is_valid(void)
{
    const tsumikoro_config_t *flash_config = (const tsumikoro_config_t *)FLASH_CONFIG_ADDR;
    return config_validate(flash_config);
}

void tsumikoro_flash_config_init_defaults(tsumikoro_config_t *config,
                                           uint8_t device_id,
                                           uint8_t hardware_id)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(tsumikoro_config_t));

    config->magic = TSUMIKORO_CONFIG_MAGIC;
    config->version = TSUMIKORO_CONFIG_VERSION;
    config->device_id = device_id;
    config->hardware_id = hardware_id;

    /* Default servo calibration (standard 1000-2000µs range) */
    config->servo_count = TSUMIKORO_CONFIG_MAX_SERVOS;
    for (uint8_t i = 0; i < TSUMIKORO_CONFIG_MAX_SERVOS; i++) {
        config->servo_cal[i].min_pulse_us = 1000;
        config->servo_cal[i].max_pulse_us = 2000;
    }

    /* Default motor settings */
    config->motor_max_speed = 1000;    /* 100% speed */
    config->motor_accel_rate = 100;    /* Moderate acceleration */

    /* CRC will be calculated when saving */
    config->crc32 = 0;
}
