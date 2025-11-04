/**
 * @file tsumikoro_flash_config.h
 * @brief Flash configuration storage for Tsumikoro devices
 *
 * Provides persistent storage of device configuration in internal flash memory.
 * Uses the last flash page for configuration data with CRC32 validation.
 *
 * Supported configurations:
 * - Device ID
 * - Servo calibration (min/max pulse widths per channel)
 * - Motor configuration
 * - Custom device-specific settings
 *
 * Memory Layout:
 * - STM32G030: Last page (Page 15, 2KB at 0x08007800-0x08007FFF)
 * - Future devices: Last page of respective flash size
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_FLASH_CONFIG_H
#define TSUMIKORO_FLASH_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration magic number for validation
 */
#define TSUMIKORO_CONFIG_MAGIC      0x544B524F  /* "TKRO" in ASCII */

/**
 * @brief Current configuration version
 */
#define TSUMIKORO_CONFIG_VERSION    1

/**
 * @brief Maximum number of servo channels supported
 */
#define TSUMIKORO_CONFIG_MAX_SERVOS 6

/**
 * @brief Servo calibration data for a single channel
 */
typedef struct {
    uint16_t min_pulse_us;  /**< Minimum pulse width in microseconds (500-2500) */
    uint16_t max_pulse_us;  /**< Maximum pulse width in microseconds (500-2500) */
} tsumikoro_servo_calibration_t;

/**
 * @brief Device configuration structure
 *
 * This structure is written to flash as-is. Keep size reasonable
 * to fit within a single flash page (2KB).
 */
typedef struct {
    uint32_t magic;         /**< Magic number for validation (TSUMIKORO_CONFIG_MAGIC) */
    uint16_t version;       /**< Configuration version */
    uint16_t reserved1;     /**< Reserved for alignment */

    /* Device identity */
    uint8_t device_id;      /**< Device ID (0x01-0xEF) */
    uint8_t hardware_id;    /**< Hardware ID read from pins (0-3) */
    uint16_t reserved2;     /**< Reserved for alignment */

    /* Servo configuration */
    uint8_t servo_count;    /**< Number of servo channels configured */
    uint8_t reserved3[3];   /**< Reserved for alignment */
    tsumikoro_servo_calibration_t servo_cal[TSUMIKORO_CONFIG_MAX_SERVOS];

    /* Motor configuration */
    uint16_t motor_max_speed;   /**< Maximum motor speed (0-1000) */
    uint16_t motor_accel_rate;  /**< Motor acceleration rate */

    /* Custom data area (expandable) */
    uint8_t custom_data[32];    /**< Custom device-specific data */

    uint32_t crc32;         /**< CRC32 of all data above (excludes this field) */
} tsumikoro_config_t;

/**
 * @brief Flash configuration error codes
 */
typedef enum {
    TSUMIKORO_FLASH_OK = 0,         /**< Operation successful */
    TSUMIKORO_FLASH_ERROR_INVALID,  /**< Invalid configuration (magic/CRC mismatch) */
    TSUMIKORO_FLASH_ERROR_ERASE,    /**< Flash erase failed */
    TSUMIKORO_FLASH_ERROR_WRITE,    /**< Flash write failed */
    TSUMIKORO_FLASH_ERROR_VERIFY,   /**< Verify failed after write */
    TSUMIKORO_FLASH_ERROR_LOCKED,   /**< Flash is locked */
} tsumikoro_flash_status_t;

/**
 * @brief Initialize flash configuration system
 *
 * Must be called before any other flash config operations.
 * Initializes HAL flash interface.
 *
 * @return TSUMIKORO_FLASH_OK on success
 */
tsumikoro_flash_status_t tsumikoro_flash_config_init(void);

/**
 * @brief Load configuration from flash
 *
 * Reads configuration from flash and validates magic number and CRC.
 * If validation fails, config is not modified.
 *
 * @param config Pointer to configuration structure to populate
 * @return TSUMIKORO_FLASH_OK on success, error code otherwise
 */
tsumikoro_flash_status_t tsumikoro_flash_config_load(tsumikoro_config_t *config);

/**
 * @brief Save configuration to flash
 *
 * Erases flash page, calculates CRC, and writes configuration.
 * Verifies write after completion.
 *
 * @param config Pointer to configuration structure to save
 * @return TSUMIKORO_FLASH_OK on success, error code otherwise
 */
tsumikoro_flash_status_t tsumikoro_flash_config_save(const tsumikoro_config_t *config);

/**
 * @brief Erase configuration from flash
 *
 * Erases the configuration flash page, resetting to factory state.
 *
 * @return TSUMIKORO_FLASH_OK on success, error code otherwise
 */
tsumikoro_flash_status_t tsumikoro_flash_config_erase(void);

/**
 * @brief Check if valid configuration exists in flash
 *
 * Validates magic number and CRC without loading full config.
 *
 * @return true if valid configuration exists, false otherwise
 */
bool tsumikoro_flash_config_is_valid(void);

/**
 * @brief Initialize configuration with defaults
 *
 * Populates config structure with default values.
 * Does not write to flash (use tsumikoro_flash_config_save for that).
 *
 * @param config Pointer to configuration structure to initialize
 * @param device_id Device ID to use (0x01-0xEF)
 * @param hardware_id Hardware ID read from pins (0-3)
 */
void tsumikoro_flash_config_init_defaults(tsumikoro_config_t *config,
                                           uint8_t device_id,
                                           uint8_t hardware_id);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_FLASH_CONFIG_H */
