/**
 * @file tsumikoro_protocol.c
 * @brief Core protocol implementation for Tsumikoro multi-drop serial bus
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tsumikoro_protocol.h"
#include "tsumikoro_crc8.h"
#include <string.h>

/**
 * @brief Write a byte to the buffer with byte stuffing
 *
 * Applies byte stuffing escape sequences for 0xAA and 0x55.
 *
 * @param buffer Output buffer
 * @param idx Current buffer index (will be updated)
 * @param byte Byte to write
 * @param buffer_len Total buffer size
 * @return true if byte was written, false if buffer full
 */
static bool write_stuffed_byte(uint8_t *buffer, size_t *idx, uint8_t byte, size_t buffer_len)
{
    if (byte == TSUMIKORO_PACKET_START) {
        // Escape 0xAA as: 0xAA 0x01
        if (*idx + 2 > buffer_len) {
            return false;
        }
        buffer[(*idx)++] = TSUMIKORO_ESCAPE_BYTE;
        buffer[(*idx)++] = TSUMIKORO_ESCAPE_AA;
    } else if (byte == TSUMIKORO_PACKET_END) {
        // Escape 0x55 as: 0xAA 0x02
        if (*idx + 2 > buffer_len) {
            return false;
        }
        buffer[(*idx)++] = TSUMIKORO_ESCAPE_BYTE;
        buffer[(*idx)++] = TSUMIKORO_ESCAPE_55;
    } else {
        // Write byte as-is
        if (*idx + 1 > buffer_len) {
            return false;
        }
        buffer[(*idx)++] = byte;
    }
    return true;
}

size_t tsumikoro_packet_encode(const tsumikoro_packet_t *packet,
                                uint8_t *buffer,
                                size_t buffer_len)
{
    // Validate inputs
    if (packet == NULL || buffer == NULL) {
        return 0;
    }

    if (packet->data_len > TSUMIKORO_MAX_DATA_LEN) {
        return 0;
    }

    // Ensure buffer is large enough for worst case (all bytes escaped)
    if (buffer_len < TSUMIKORO_MIN_PACKET_LEN) {
        return 0;
    }

    size_t idx = 0;

    // Write START marker (2 bytes: 0xAA 0xAA)
    if (idx + 2 > buffer_len) {
        return 0;
    }
    buffer[idx++] = TSUMIKORO_PACKET_START;
    buffer[idx++] = TSUMIKORO_PACKET_START;

    // Build unstuffed payload in temporary buffer first to calculate CRC
    uint8_t unstuffed_payload[TSUMIKORO_MAX_DATA_LEN + 5];  // ID + CMD_HI + CMD_LO + LEN + DATA
    size_t unstuffed_idx = 0;

    unstuffed_payload[unstuffed_idx++] = packet->device_id;
    unstuffed_payload[unstuffed_idx++] = (packet->command >> 8) & 0xFF;  // CMD_HI
    unstuffed_payload[unstuffed_idx++] = packet->command & 0xFF;         // CMD_LO
    unstuffed_payload[unstuffed_idx++] = packet->data_len;

    if (packet->data_len > 0) {
        memcpy(&unstuffed_payload[unstuffed_idx], packet->data, packet->data_len);
        unstuffed_idx += packet->data_len;
    }

    // Calculate CRC8 over unstuffed payload
    uint8_t crc = tsumikoro_crc8_calculate_table(unstuffed_payload, unstuffed_idx);

    // Now write stuffed payload to output buffer
    for (size_t i = 0; i < unstuffed_idx; i++) {
        if (!write_stuffed_byte(buffer, &idx, unstuffed_payload[i], buffer_len)) {
            return 0;  // Buffer overflow
        }
    }

    // Write stuffed CRC
    if (!write_stuffed_byte(buffer, &idx, crc, buffer_len)) {
        return 0;
    }

    // Write END marker (not stuffed, it's the delimiter)
    if (idx + 1 > buffer_len) {
        return 0;
    }
    buffer[idx++] = TSUMIKORO_PACKET_END;

    return idx;  // Return total bytes written
}

tsumikoro_status_t tsumikoro_packet_decode(const uint8_t *buffer,
                                            size_t buffer_len,
                                            tsumikoro_packet_t *packet,
                                            size_t *bytes_consumed)
{
    // Validate inputs
    if (buffer == NULL || packet == NULL || bytes_consumed == NULL) {
        return TSUMIKORO_STATUS_INVALID_PARAM;
    }

    // Initialize bytes_consumed
    *bytes_consumed = 0;

    // Check minimum packet size
    if (buffer_len < TSUMIKORO_MIN_PACKET_LEN) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Search for START marker (0xAA 0xAA) - discard any preceding bytes
    size_t start_idx = 0;
    while (start_idx + 1 < buffer_len) {
        if (buffer[start_idx] == TSUMIKORO_PACKET_START &&
            buffer[start_idx + 1] == TSUMIKORO_PACKET_START) {
            // Found start marker
            break;
        }
        start_idx++;
    }

    // Check if we found a start marker and have enough bytes remaining
    if (start_idx + 1 >= buffer_len || (buffer_len - start_idx) < TSUMIKORO_MIN_PACKET_LEN) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Search for END marker (0x55) to verify we have a complete packet
    // Note: We need to check for escaped 0x55 (AA 02) vs actual end marker
    bool found_end = false;
    size_t end_idx = start_idx + 2;  // Skip past start marker
    while (end_idx < buffer_len) {
        if (buffer[end_idx] == TSUMIKORO_PACKET_END) {
            // Check if this is an escaped 0x55 (preceded by 0xAA)
            if (end_idx > start_idx + 2 && buffer[end_idx - 1] == TSUMIKORO_ESCAPE_BYTE) {
                // This is an escape sequence, not the end marker
                end_idx++;
                continue;
            }
            // Found genuine END marker
            found_end = true;
            break;
        }
        end_idx++;
    }

    // If we don't have a complete packet yet, return not ready
    if (!found_end) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Now decode the complete packet
    size_t idx = start_idx + 2;  // Skip START marker

    // Unstuff payload into temporary buffer
    uint8_t unstuffed_payload[TSUMIKORO_MAX_DATA_LEN + 6];  // ID + CMD + LEN + DATA + CRC
    size_t unstuffed_idx = 0;

    // Read and unstuff bytes until we hit END marker
    while (idx < buffer_len) {
        uint8_t byte = buffer[idx++];

        if (byte == TSUMIKORO_PACKET_END) {
            // Found END marker, done reading payload
            break;
        } else if (byte == TSUMIKORO_ESCAPE_BYTE) {
            // Escape sequence
            if (idx >= buffer_len) {
                return TSUMIKORO_STATUS_INVALID_PACKET;  // Incomplete escape sequence
            }

            uint8_t next_byte = buffer[idx++];
            if (next_byte == TSUMIKORO_ESCAPE_AA) {
                // 0xAA 0x01 → 0xAA
                unstuffed_payload[unstuffed_idx++] = TSUMIKORO_PACKET_START;
            } else if (next_byte == TSUMIKORO_ESCAPE_55) {
                // 0xAA 0x02 → 0x55
                unstuffed_payload[unstuffed_idx++] = TSUMIKORO_PACKET_END;
            } else if (next_byte == TSUMIKORO_PACKET_START) {
                // 0xAA 0xAA → Start of new frame, abort current frame
                return TSUMIKORO_STATUS_INVALID_PACKET;
            } else {
                // Invalid escape sequence
                return TSUMIKORO_STATUS_INVALID_PACKET;
            }

            if (unstuffed_idx > sizeof(unstuffed_payload)) {
                return TSUMIKORO_STATUS_INVALID_PACKET;  // Overflow
            }
        } else {
            // Regular byte
            unstuffed_payload[unstuffed_idx++] = byte;

            if (unstuffed_idx > sizeof(unstuffed_payload)) {
                return TSUMIKORO_STATUS_INVALID_PACKET;  // Overflow
            }
        }
    }

    // Minimum unstuffed payload: ID(1) + CMD_HI(1) + CMD_LO(1) + LEN(1) + CRC(1) = 5 bytes
    if (unstuffed_idx < 5) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Parse unstuffed payload
    size_t payload_idx = 0;

    // Extract device ID
    packet->device_id = unstuffed_payload[payload_idx++];

    // Extract command (16-bit, big-endian)
    packet->command = ((uint16_t)unstuffed_payload[payload_idx] << 8) |
                      unstuffed_payload[payload_idx + 1];
    payload_idx += 2;

    // Extract data length
    packet->data_len = unstuffed_payload[payload_idx++];

    // Validate data length
    if (packet->data_len > TSUMIKORO_MAX_DATA_LEN) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Check we have enough bytes for data + CRC
    if (unstuffed_idx < (payload_idx + packet->data_len + 1)) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Extract data payload
    if (packet->data_len > 0) {
        memcpy(packet->data, &unstuffed_payload[payload_idx], packet->data_len);
        payload_idx += packet->data_len;
    }

    // Extract received CRC
    uint8_t received_crc = unstuffed_payload[payload_idx];

    // Calculate CRC over ID + CMD_HI + CMD_LO + LEN + DATA
    uint8_t calculated_crc = tsumikoro_crc8_calculate_table(unstuffed_payload, payload_idx);

    if (received_crc != calculated_crc) {
        return TSUMIKORO_STATUS_CRC_ERROR;
    }

    // Set bytes consumed: everything up to and including the END marker
    *bytes_consumed = end_idx + 1;

    return TSUMIKORO_STATUS_OK;
}
