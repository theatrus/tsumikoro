/**
 * @file tsumikoro_protocol.h
 * @brief Shared protocol definitions for Tsumikoro motor control system
 *
 * This header is shared between STM32 firmware and ESP32 bridge code,
 * defining the common communication protocol.
 *
 * Copyright (c) 2025-2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_PROTOCOL_H
#define TSUMIKORO_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Protocol version */
#define TSUMIKORO_PROTOCOL_VERSION 1

/** Packet delimiters */
#define TSUMIKORO_PACKET_START 0xAA
#define TSUMIKORO_PACKET_END   0x55

/** Maximum packet data length */
#define TSUMIKORO_MAX_DATA_LEN 64

/** Command definitions */
typedef enum {
    TSUMIKORO_CMD_SET_SPEED    = 0x01,  /**< Set motor speed */
    TSUMIKORO_CMD_GET_STATUS   = 0x02,  /**< Get controller status */
    TSUMIKORO_CMD_STOP         = 0x03,  /**< Emergency stop */
    TSUMIKORO_CMD_RESET        = 0x04,  /**< Reset controller */
    TSUMIKORO_CMD_SET_POSITION = 0x05,  /**< Set target position (stepper) */
    TSUMIKORO_CMD_GET_POSITION = 0x06,  /**< Get current position */
    TSUMIKORO_CMD_SET_CONFIG   = 0x07,  /**< Set configuration */
    TSUMIKORO_CMD_GET_CONFIG   = 0x08,  /**< Get configuration */
} tsumikoro_command_t;

/** Response/Status codes */
typedef enum {
    TSUMIKORO_STATUS_OK          = 0x00,  /**< Command successful */
    TSUMIKORO_STATUS_ERROR       = 0x01,  /**< Generic error */
    TSUMIKORO_STATUS_INVALID_CMD = 0x02,  /**< Invalid command */
    TSUMIKORO_STATUS_INVALID_DATA= 0x03,  /**< Invalid data */
    TSUMIKORO_STATUS_BUSY        = 0x04,  /**< Controller busy */
    TSUMIKORO_STATUS_FAULT       = 0x05,  /**< Fault condition */
} tsumikoro_status_t;

/** Controller state flags */
typedef enum {
    TSUMIKORO_STATE_IDLE     = 0x00,
    TSUMIKORO_STATE_RUNNING  = 0x01,
    TSUMIKORO_STATE_STOPPED  = 0x02,
    TSUMIKORO_STATE_ERROR    = 0x04,
    TSUMIKORO_STATE_MOVING   = 0x08,  /**< For stepper: moving to position */
} tsumikoro_state_t;

/**
 * @brief Packet structure
 *
 * Wire format:
 * [START(1)][ID(1)][CMD(1)][LEN(1)][DATA(0-64)][CHECKSUM(1)][END(1)]
 */
typedef struct {
    uint8_t start;           /**< Start marker (TSUMIKORO_PACKET_START) */
    uint8_t controller_id;   /**< Controller ID (0-255) */
    uint8_t command;         /**< Command byte */
    uint8_t data_len;        /**< Length of data field */
    uint8_t data[TSUMIKORO_MAX_DATA_LEN];  /**< Command data */
    uint8_t checksum;        /**< XOR checksum of ID through DATA */
    uint8_t end;             /**< End marker (TSUMIKORO_PACKET_END) */
} tsumikoro_packet_t;

/**
 * @brief Status response data
 */
typedef struct {
    uint8_t state;           /**< Current state (tsumikoro_state_t) */
    int16_t speed;           /**< Current speed (-32768 to 32767) */
    int32_t position;        /**< Current position (for steppers) */
    uint16_t current;        /**< Motor current (mA) */
    uint8_t temperature;     /**< Controller temperature (°C) */
    uint8_t fault_flags;     /**< Fault condition flags */
} tsumikoro_status_data_t;

/**
 * @brief Calculate XOR checksum for packet
 *
 * @param data Pointer to data starting from controller_id
 * @param len Length of data (ID + CMD + LEN + DATA)
 * @return Checksum value
 */
static inline uint8_t tsumikoro_calculate_checksum(const uint8_t *data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Validate packet checksum
 *
 * @param packet Pointer to packet structure
 * @return true if checksum is valid
 */
static inline bool tsumikoro_validate_packet(const tsumikoro_packet_t *packet) {
    if (packet->start != TSUMIKORO_PACKET_START) return false;
    if (packet->end != TSUMIKORO_PACKET_END) return false;
    if (packet->data_len > TSUMIKORO_MAX_DATA_LEN) return false;

    uint8_t calc_checksum = tsumikoro_calculate_checksum(
        (const uint8_t *)&packet->controller_id,
        3 + packet->data_len  /* ID + CMD + LEN + DATA */
    );

    return (calc_checksum == packet->checksum);
}

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_PROTOCOL_H */
