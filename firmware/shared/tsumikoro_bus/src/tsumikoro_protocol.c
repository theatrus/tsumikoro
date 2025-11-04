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

    // Calculate required buffer size
    size_t packet_size = tsumikoro_packet_size(packet->data_len);
    if (buffer_len < packet_size) {
        return 0;
    }

    // Build packet in buffer
    size_t idx = 0;

    // START marker
    buffer[idx++] = TSUMIKORO_PACKET_START;

    // Device ID
    buffer[idx++] = packet->device_id;

    // Command (16-bit, big-endian)
    buffer[idx++] = (packet->command >> 8) & 0xFF;  // CMD_HI
    buffer[idx++] = packet->command & 0xFF;         // CMD_LO

    // Data length
    buffer[idx++] = packet->data_len;

    // Data payload
    if (packet->data_len > 0) {
        memcpy(&buffer[idx], packet->data, packet->data_len);
        idx += packet->data_len;
    }

    // Calculate CRC8 over ID + CMD_HI + CMD_LO + LEN + DATA
    // (everything between START and CRC, excluding markers)
    uint8_t crc = tsumikoro_crc8_calculate_table(&buffer[1], idx - 1);
    buffer[idx++] = crc;

    // END marker
    buffer[idx++] = TSUMIKORO_PACKET_END;

    return idx;  // Return total bytes written
}

tsumikoro_status_t tsumikoro_packet_decode(const uint8_t *buffer,
                                            size_t buffer_len,
                                            tsumikoro_packet_t *packet)
{
    // Validate inputs
    if (buffer == NULL || packet == NULL) {
        return TSUMIKORO_STATUS_INVALID_PARAM;
    }

    // Check minimum packet size
    if (buffer_len < TSUMIKORO_MIN_PACKET_LEN) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    size_t idx = 0;

    // Verify START marker
    if (buffer[idx++] != TSUMIKORO_PACKET_START) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Extract device ID
    packet->device_id = buffer[idx++];

    // Extract command (16-bit, big-endian)
    packet->command = ((uint16_t)buffer[idx] << 8) | buffer[idx + 1];
    idx += 2;

    // Extract data length
    packet->data_len = buffer[idx++];

    // Validate data length
    if (packet->data_len > TSUMIKORO_MAX_DATA_LEN) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Check if buffer has enough bytes for data + CRC + END
    size_t expected_len = tsumikoro_packet_size(packet->data_len);
    if (buffer_len < expected_len) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    // Extract data payload
    if (packet->data_len > 0) {
        memcpy(packet->data, &buffer[idx], packet->data_len);
        idx += packet->data_len;
    }

    // Extract and verify CRC8
    uint8_t received_crc = buffer[idx++];

    // Calculate CRC over ID + CMD_HI + CMD_LO + LEN + DATA
    uint8_t calculated_crc = tsumikoro_crc8_calculate_table(&buffer[1], idx - 2);

    if (received_crc != calculated_crc) {
        return TSUMIKORO_STATUS_CRC_ERROR;
    }

    // Verify END marker
    if (buffer[idx] != TSUMIKORO_PACKET_END) {
        return TSUMIKORO_STATUS_INVALID_PACKET;
    }

    return TSUMIKORO_STATUS_OK;
}
