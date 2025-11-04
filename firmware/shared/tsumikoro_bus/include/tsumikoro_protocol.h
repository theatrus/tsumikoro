/**
 * @file tsumikoro_protocol.h
 * @brief Core protocol definitions for Tsumikoro multi-drop serial bus
 *
 * Wire Format:
 * [START(1)][ID(1)][CMD_HI(1)][CMD_LO(1)][LEN(1)][DATA(0-64)][CRC8(1)][END(1)]
 *
 * Packet size: 7-71 bytes
 * - Minimum (no data): 7 bytes
 * - Maximum (64 bytes data): 71 bytes
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_PROTOCOL_H
#define TSUMIKORO_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Protocol version
 */
#define TSUMIKORO_PROTOCOL_VERSION_MAJOR 0
#define TSUMIKORO_PROTOCOL_VERSION_MINOR 6

/**
 * @brief Packet framing markers
 */
#define TSUMIKORO_PACKET_START  0xAA
#define TSUMIKORO_PACKET_END    0x55

/**
 * @brief Packet size limits
 */
#define TSUMIKORO_MAX_DATA_LEN      64    /**< Maximum data payload size */
#define TSUMIKORO_MIN_PACKET_LEN    7     /**< Minimum packet: START+ID+CMD_HI+CMD_LO+LEN+CRC+END */
#define TSUMIKORO_MAX_PACKET_LEN    71    /**< Maximum packet with 64 bytes data */

/**
 * @brief Device address ranges
 */
#define TSUMIKORO_ADDR_CONTROLLER   0x00  /**< Controller/bridge address */
#define TSUMIKORO_ADDR_MIN          0x01  /**< Minimum peripheral address */
#define TSUMIKORO_ADDR_MAX          0xEF  /**< Maximum peripheral address */
#define TSUMIKORO_ADDR_RESERVED_MIN 0xF0  /**< Reserved range start */
#define TSUMIKORO_ADDR_RESERVED_MAX 0xFE  /**< Reserved range end */
#define TSUMIKORO_ADDR_BROADCAST    0xFF  /**< Broadcast address */

/**
 * @brief Generic commands (0x0000-0x0FFF)
 */
#define TSUMIKORO_CMD_PING                0x0001  /**< Ping device */
#define TSUMIKORO_CMD_GET_INFO            0x0002  /**< Get device information */
#define TSUMIKORO_CMD_STOP                0x0003  /**< Emergency stop */
#define TSUMIKORO_CMD_RESET               0x0004  /**< Reset device */
#define TSUMIKORO_CMD_BOOTLOADER          0x0005  /**< Enter bootloader mode */
#define TSUMIKORO_CMD_GET_VERSION         0x0006  /**< Get firmware version */
#define TSUMIKORO_CMD_SET_DEVICE_ID       0x0007  /**< Change device ID */
#define TSUMIKORO_CMD_SAVE_CONFIG         0x0008  /**< Save configuration to flash */
#define TSUMIKORO_CMD_LOAD_CONFIG         0x0009  /**< Load configuration from flash */
#define TSUMIKORO_CMD_GET_STATUS          0x000A  /**< Get device status */
#define TSUMIKORO_CMD_ID_ASSIGN_START     0x000B  /**< Start hardware ID assignment (OPTIONAL) */
#define TSUMIKORO_CMD_ID_ASSIGN_ACK       0x000C  /**< Peripheral acknowledges assigned ID (OPTIONAL) */
#define TSUMIKORO_CMD_ID_ASSIGN_COMPLETE  0x000D  /**< ID assignment complete (OPTIONAL) */

/**
 * @brief Command range definitions
 */
#define TSUMIKORO_CMD_RANGE_GENERIC_MIN   0x0000  /**< Generic commands start */
#define TSUMIKORO_CMD_RANGE_GENERIC_MAX   0x0FFF  /**< Generic commands end */
#define TSUMIKORO_CMD_RANGE_SENSOR_MIN    0x1000  /**< Sensor commands start */
#define TSUMIKORO_CMD_RANGE_SENSOR_MAX    0x1FFF  /**< Sensor commands end */
#define TSUMIKORO_CMD_RANGE_STEPPER_MIN   0x2000  /**< Stepper commands start */
#define TSUMIKORO_CMD_RANGE_STEPPER_MAX   0x2FFF  /**< Stepper commands end */
#define TSUMIKORO_CMD_RANGE_SERVO_MIN     0x3000  /**< Servo commands start */
#define TSUMIKORO_CMD_RANGE_SERVO_MAX     0x3FFF  /**< Servo commands end */
#define TSUMIKORO_CMD_RANGE_DCMOTOR_MIN   0x4000  /**< DC motor commands start */
#define TSUMIKORO_CMD_RANGE_DCMOTOR_MAX   0x4FFF  /**< DC motor commands end */
#define TSUMIKORO_CMD_RANGE_BRIDGE_MIN    0x5000  /**< Bridge commands start */
#define TSUMIKORO_CMD_RANGE_BRIDGE_MAX    0x5FFF  /**< Bridge commands end */
#define TSUMIKORO_CMD_RANGE_CUSTOM_MIN    0xF000  /**< Custom commands start */
#define TSUMIKORO_CMD_RANGE_CUSTOM_MAX    0xFFFF  /**< Custom commands end */

/**
 * @brief Status codes
 */
typedef enum {
    TSUMIKORO_STATUS_OK = 0x00,           /**< Success */
    TSUMIKORO_STATUS_ERROR = 0x01,        /**< Generic error */
    TSUMIKORO_STATUS_INVALID_PACKET = 0x02,    /**< Invalid packet format */
    TSUMIKORO_STATUS_CRC_ERROR = 0x03,    /**< CRC check failed */
    TSUMIKORO_STATUS_TIMEOUT = 0x04,      /**< Operation timeout */
    TSUMIKORO_STATUS_BUSY = 0x05,         /**< Device busy */
    TSUMIKORO_STATUS_NOT_IMPLEMENTED = 0x06,  /**< Command not implemented */
    TSUMIKORO_STATUS_INVALID_PARAM = 0x07,    /**< Invalid parameter */
    TSUMIKORO_STATUS_BUFFER_FULL = 0x08,  /**< Buffer overflow */
    TSUMIKORO_STATUS_NAK = 0x09,          /**< Command rejected (NAK) */
} tsumikoro_status_t;

/**
 * @brief Packet structure (wire format)
 *
 * This structure represents a packet in memory. The wire format
 * is serialized/deserialized by encode/decode functions.
 */
typedef struct {
    uint8_t device_id;                    /**< Device ID (destination or source) */
    uint16_t command;                     /**< 16-bit command */
    uint8_t data_len;                     /**< Data length (0-64) */
    uint8_t data[TSUMIKORO_MAX_DATA_LEN]; /**< Data payload */
} tsumikoro_packet_t;

/**
 * @brief Encode a packet into wire format
 *
 * Converts a packet structure into the wire format with framing,
 * CRC8, and proper byte ordering.
 *
 * Wire format:
 * [START][ID][CMD_HI][CMD_LO][LEN][DATA...][CRC8][END]
 *
 * @param packet Pointer to packet structure
 * @param buffer Output buffer for encoded packet
 * @param buffer_len Size of output buffer (must be >= packet size)
 * @return Number of bytes written to buffer, or 0 on error
 */
size_t tsumikoro_packet_encode(const tsumikoro_packet_t *packet,
                                uint8_t *buffer,
                                size_t buffer_len);

/**
 * @brief Decode a packet from wire format
 *
 * Parses wire format data and converts to packet structure.
 * Validates framing markers and CRC8.
 *
 * @param buffer Input buffer containing encoded packet
 * @param buffer_len Length of input buffer
 * @param packet Output packet structure
 * @return TSUMIKORO_STATUS_OK on success, error code otherwise
 */
tsumikoro_status_t tsumikoro_packet_decode(const uint8_t *buffer,
                                            size_t buffer_len,
                                            tsumikoro_packet_t *packet);

/**
 * @brief Calculate packet size for given data length
 *
 * @param data_len Data payload length (0-64)
 * @return Total packet size in bytes (7-71)
 */
static inline size_t tsumikoro_packet_size(uint8_t data_len)
{
    return 7 + data_len;  // START + ID + CMD_HI + CMD_LO + LEN + DATA + CRC + END
}

/**
 * @brief Check if device ID is valid
 *
 * @param device_id Device ID to check
 * @return true if valid, false otherwise
 */
static inline bool tsumikoro_is_valid_device_id(uint8_t device_id)
{
    return (device_id == TSUMIKORO_ADDR_CONTROLLER) ||
           (device_id >= TSUMIKORO_ADDR_MIN && device_id <= TSUMIKORO_ADDR_MAX) ||
           (device_id == TSUMIKORO_ADDR_BROADCAST);
}

/**
 * @brief Check if command is in generic range
 *
 * @param command Command to check
 * @return true if generic command, false otherwise
 */
static inline bool tsumikoro_is_generic_command(uint16_t command)
{
    // Note: command >= 0x0000 is always true for uint16_t, so only check upper bound
    return (command <= TSUMIKORO_CMD_RANGE_GENERIC_MAX);
}

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_PROTOCOL_H */
