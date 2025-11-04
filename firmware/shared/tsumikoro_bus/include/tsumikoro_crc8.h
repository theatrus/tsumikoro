/**
 * @file tsumikoro_crc8.h
 * @brief CRC8-CCITT implementation for Tsumikoro bus protocol
 *
 * Polynomial: 0x07 (x^8 + x^2 + x + 1)
 * Initial value: 0x00
 *
 * Provides 100% detection of:
 * - All single-bit errors
 * - All double-bit errors
 * - All burst errors ≤8 bits
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_CRC8_H
#define TSUMIKORO_CRC8_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CRC8-CCITT polynomial (x^8 + x^2 + x + 1)
 */
#define TSUMIKORO_CRC8_POLY 0x07

/**
 * @brief Calculate CRC8-CCITT for a block of data
 *
 * This is a bitwise implementation suitable for resource-constrained
 * devices (STM32G030 with 32KB flash). For larger devices, consider
 * using the table-based implementation.
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC8 checksum
 */
uint8_t tsumikoro_crc8_calculate(const uint8_t *data, size_t len);

/**
 * @brief Calculate CRC8-CCITT using lookup table (faster)
 *
 * Uses 256-byte lookup table for faster computation.
 * Recommended for STM32G071 and ESP32 with more flash available.
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return CRC8 checksum
 */
uint8_t tsumikoro_crc8_calculate_table(const uint8_t *data, size_t len);

/**
 * @brief Verify CRC8 checksum
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @param expected_crc Expected CRC8 value
 * @return 1 if CRC matches, 0 otherwise
 */
int tsumikoro_crc8_verify(const uint8_t *data, size_t len, uint8_t expected_crc);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_CRC8_H */
