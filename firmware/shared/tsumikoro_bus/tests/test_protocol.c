/**
 * @file test_protocol.c
 * @brief Unit tests for Tsumikoro bus protocol encoding/decoding
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_framework.h"
#include "tsumikoro_protocol.h"
#include "tsumikoro_crc8.h"

/**
 * Test: Encode minimum packet (no data)
 */
void test_encode_minimum_packet(void)
{
    tsumikoro_packet_t packet = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t len = tsumikoro_packet_encode(&packet, buffer, sizeof(buffer));

    // Should be 8 bytes: START(2) + ID + CMD_HI + CMD_LO + LEN + CRC + END
    TEST_ASSERT_EQUAL(8, len);

    // Verify structure
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[0]);  // START 1
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[1]);  // START 2
    TEST_ASSERT_EQUAL(0x01, buffer[2]);                     // ID
    TEST_ASSERT_EQUAL(0x00, buffer[3]);                     // CMD_HI
    TEST_ASSERT_EQUAL(0x01, buffer[4]);                     // CMD_LO
    TEST_ASSERT_EQUAL(0x00, buffer[5]);                     // LEN
    // buffer[6] is CRC
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_END, buffer[7]);     // END

    printf("    Encoded packet: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

/**
 * Test: Encode packet with data
 */
void test_encode_packet_with_data(void)
{
    tsumikoro_packet_t packet = {
        .device_id = 0x02,
        .command = TSUMIKORO_CMD_GET_STATUS,
        .data_len = 4,
        .data = { 0x10, 0x20, 0x30, 0x40 }
    };

    uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t len = tsumikoro_packet_encode(&packet, buffer, sizeof(buffer));

    // Should be 12 bytes: START(2) + ID + CMD_HI + CMD_LO + LEN + 4 DATA + CRC + END
    TEST_ASSERT_EQUAL(12, len);

    // Verify structure
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[0]);  // START 1
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[1]);  // START 2
    TEST_ASSERT_EQUAL(0x02, buffer[2]);
    TEST_ASSERT_EQUAL(0x00, buffer[3]);  // CMD_HI
    TEST_ASSERT_EQUAL(0x0A, buffer[4]);  // CMD_LO (0x000A = CMD_GET_STATUS)
    TEST_ASSERT_EQUAL(0x04, buffer[5]);  // LEN
    TEST_ASSERT_EQUAL(0x10, buffer[6]);  // DATA[0]
    TEST_ASSERT_EQUAL(0x20, buffer[7]);  // DATA[1]
    TEST_ASSERT_EQUAL(0x30, buffer[8]);  // DATA[2]
    TEST_ASSERT_EQUAL(0x40, buffer[9]);  // DATA[3]
    // buffer[10] is CRC
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_END, buffer[11]);

    printf("    Encoded packet: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

/**
 * Test: Decode minimum packet
 */
void test_decode_minimum_packet(void)
{
    // Manually constructed packet: PING to device 0x01
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,  // START 1
        TSUMIKORO_PACKET_START,  // START 2
        0x01,                     // ID
        0x00,                     // CMD_HI
        0x01,                     // CMD_LO (PING)
        0x00,                     // LEN
        0x00,                     // CRC (placeholder, will calculate)
        TSUMIKORO_PACKET_END      // END
    };

    // Calculate correct CRC (over bytes 2-5: ID + CMD_HI + CMD_LO + LEN)
    buffer[6] = tsumikoro_crc8_calculate_table(&buffer[2], 4);

    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(0x01, packet.device_id);
    TEST_ASSERT_EQUAL(TSUMIKORO_CMD_PING, packet.command);
    TEST_ASSERT_EQUAL(0, packet.data_len);
    TEST_ASSERT_EQUAL(8, bytes_consumed);  // Full packet consumed
}

/**
 * Test: Decode packet with data (including byte stuffing test)
 */
void test_decode_packet_with_data(void)
{
    // Create packet with data that includes 0xAA (will be escaped)
    tsumikoro_packet_t original = {
        .device_id = 0x03,
        .command = TSUMIKORO_CMD_GET_INFO,
        .data_len = 3,
        .data = { 0xAA, 0xBB, 0xCC }  // 0xAA will be escaped in wire format
    };

    // Encode to get properly stuffed wire format
    uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t len = tsumikoro_packet_encode(&original, buffer, sizeof(buffer));

    // Length should be 12 or 13 bytes depending on whether CRC needs escaping
    // START(2) + ID + CMD_HI + CMD_LO + LEN + (0xAA->0xAA 0x01) + BB + CC + CRC(1-2) + END
    TEST_ASSERT(len >= 12 && len <= 13, "Encoded length should be 12-13 bytes");

    // Decode the packet
    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, len, &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(0x03, packet.device_id);
    TEST_ASSERT_EQUAL(TSUMIKORO_CMD_GET_INFO, packet.command);
    TEST_ASSERT_EQUAL(3, packet.data_len);
    TEST_ASSERT_EQUAL(0xAA, packet.data[0]);  // Should be unstuffed correctly
    TEST_ASSERT_EQUAL(0xBB, packet.data[1]);
    TEST_ASSERT_EQUAL(0xCC, packet.data[2]);
    TEST_ASSERT_EQUAL(len, bytes_consumed);
}

/**
 * Test: Roundtrip encode/decode
 */
void test_roundtrip_encode_decode(void)
{
    // Create original packet
    tsumikoro_packet_t original = {
        .device_id = 0xFF,  // Broadcast
        .command = TSUMIKORO_CMD_STOP,
        .data_len = 2,
        .data = { 0x12, 0x34 }
    };

    // Encode
    uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t len = tsumikoro_packet_encode(&original, buffer, sizeof(buffer));
    TEST_ASSERT(len > 0, "Encode should succeed");

    // Decode
    tsumikoro_packet_t decoded;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, len, &decoded, &bytes_consumed);

    // Verify
    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(original.device_id, decoded.device_id);
    TEST_ASSERT_EQUAL(original.command, decoded.command);
    TEST_ASSERT_EQUAL(original.data_len, decoded.data_len);
    TEST_ASSERT_EQUAL(len, bytes_consumed);

    if (original.data_len > 0) {
        TEST_ASSERT_EQUAL_BYTES(original.data, decoded.data, original.data_len);
    }
}

/**
 * Test: CRC error detection
 */
void test_decode_crc_error(void)
{
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,  // START 1
        TSUMIKORO_PACKET_START,  // START 2
        0x01,
        0x00,
        0x01,
        0x00,
        0xFF,  // Incorrect CRC
        TSUMIKORO_PACKET_END
    };

    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_CRC_ERROR, status);
}

/**
 * Test: Invalid START marker
 */
void test_decode_invalid_start(void)
{
    uint8_t buffer[] = {
        0x00,  // Wrong START marker
        0x01,
        0x00,
        0x01,
        0x00,
        0x00,
        TSUMIKORO_PACKET_END
    };

    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_INVALID_PACKET, status);
}

/**
 * Test: Invalid END marker
 */
void test_decode_invalid_end(void)
{
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,  // START 1
        TSUMIKORO_PACKET_START,  // START 2
        0x01,
        0x00,
        0x01,
        0x00,
        0x00,
        0x00  // Wrong END marker
    };

    // Calculate correct CRC
    buffer[6] = tsumikoro_crc8_calculate_table(&buffer[2], 4);

    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_INVALID_PACKET, status);
}

/**
 * Test: Buffer too small
 */
void test_decode_buffer_too_small(void)
{
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,  // START 1
        TSUMIKORO_PACKET_START,  // START 2
        0x01,
        0x00,
        0x01
        // Truncated packet
    };

    tsumikoro_packet_t packet;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_INVALID_PACKET, status);
}

/**
 * Test: Maximum data length
 */
void test_encode_max_data(void)
{
    tsumikoro_packet_t packet = {
        .device_id = 0x10,
        .command = 0x1234,
        .data_len = TSUMIKORO_MAX_DATA_LEN
    };

    // Fill data with pattern
    for (int i = 0; i < TSUMIKORO_MAX_DATA_LEN; i++) {
        packet.data[i] = (uint8_t)i;
    }

    uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t len = tsumikoro_packet_encode(&packet, buffer, sizeof(buffer));

    // Should encode successfully (size depends on whether any bytes need escaping)
    TEST_ASSERT(len > 0, "Encode should succeed");
    TEST_ASSERT(len <= TSUMIKORO_MAX_PACKET_LEN, "Should fit in max packet size");

    // Verify decode
    tsumikoro_packet_t decoded;
    size_t bytes_consumed = 0;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, len, &decoded, &bytes_consumed);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(TSUMIKORO_MAX_DATA_LEN, decoded.data_len);
    TEST_ASSERT_EQUAL_BYTES(packet.data, decoded.data, TSUMIKORO_MAX_DATA_LEN);
    TEST_ASSERT_EQUAL(len, bytes_consumed);
}

/**
 * Test: Inline helper functions
 */
void test_helper_functions(void)
{
    // Test packet size calculation (minimum size without escaping)
    TEST_ASSERT_EQUAL(8, tsumikoro_packet_size(0));   // 8 bytes minimum
    TEST_ASSERT_EQUAL(9, tsumikoro_packet_size(1));   // 9 bytes with 1 data byte
    TEST_ASSERT_EQUAL(72, tsumikoro_packet_size(64)); // 72 bytes with max data

    // Test device ID validation
    TEST_ASSERT(tsumikoro_is_valid_device_id(0x00), "Controller ID should be valid");
    TEST_ASSERT(tsumikoro_is_valid_device_id(0x01), "Peripheral ID should be valid");
    TEST_ASSERT(tsumikoro_is_valid_device_id(0xEF), "Max peripheral ID should be valid");
    TEST_ASSERT(tsumikoro_is_valid_device_id(0xFF), "Broadcast ID should be valid");
    TEST_ASSERT(!tsumikoro_is_valid_device_id(0xF0), "Reserved ID should be invalid");

    // Test generic command check
    TEST_ASSERT(tsumikoro_is_generic_command(0x0001), "PING is generic");
    TEST_ASSERT(tsumikoro_is_generic_command(0x000A), "GET_STATUS is generic");
    TEST_ASSERT(!tsumikoro_is_generic_command(0x1000), "Sensor commands are not generic");
    TEST_ASSERT(!tsumikoro_is_generic_command(0x2000), "Stepper commands are not generic");
}

/**
 * Main test runner
 */
int main(void)
{
    TEST_SUITE("Protocol Encoding/Decoding Tests");

    RUN_TEST(test_encode_minimum_packet);
    RUN_TEST(test_encode_packet_with_data);
    RUN_TEST(test_decode_minimum_packet);
    RUN_TEST(test_decode_packet_with_data);
    RUN_TEST(test_roundtrip_encode_decode);
    RUN_TEST(test_decode_crc_error);
    RUN_TEST(test_decode_invalid_start);
    RUN_TEST(test_decode_invalid_end);
    RUN_TEST(test_decode_buffer_too_small);
    RUN_TEST(test_encode_max_data);
    RUN_TEST(test_helper_functions);

    TEST_SUMMARY();
    return TEST_EXIT();
}
