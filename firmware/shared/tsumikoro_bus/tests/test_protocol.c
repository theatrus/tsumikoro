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

    // Should be 7 bytes: START + ID + CMD_HI + CMD_LO + LEN + CRC + END
    TEST_ASSERT_EQUAL(7, len);

    // Verify structure
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[0]);  // START
    TEST_ASSERT_EQUAL(0x01, buffer[1]);                     // ID
    TEST_ASSERT_EQUAL(0x00, buffer[2]);                     // CMD_HI
    TEST_ASSERT_EQUAL(0x01, buffer[3]);                     // CMD_LO
    TEST_ASSERT_EQUAL(0x00, buffer[4]);                     // LEN
    // buffer[5] is CRC
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_END, buffer[6]);     // END

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

    // Should be 11 bytes: START + ID + CMD_HI + CMD_LO + LEN + 4 DATA + CRC + END
    TEST_ASSERT_EQUAL(11, len);

    // Verify structure
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_START, buffer[0]);
    TEST_ASSERT_EQUAL(0x02, buffer[1]);
    TEST_ASSERT_EQUAL(0x00, buffer[2]);  // CMD_HI
    TEST_ASSERT_EQUAL(0x0A, buffer[3]);  // CMD_LO (0x000A = CMD_GET_STATUS)
    TEST_ASSERT_EQUAL(0x04, buffer[4]);  // LEN
    TEST_ASSERT_EQUAL(0x10, buffer[5]);  // DATA[0]
    TEST_ASSERT_EQUAL(0x20, buffer[6]);  // DATA[1]
    TEST_ASSERT_EQUAL(0x30, buffer[7]);  // DATA[2]
    TEST_ASSERT_EQUAL(0x40, buffer[8]);  // DATA[3]
    // buffer[9] is CRC
    TEST_ASSERT_EQUAL(TSUMIKORO_PACKET_END, buffer[10]);

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
        TSUMIKORO_PACKET_START,  // START
        0x01,                     // ID
        0x00,                     // CMD_HI
        0x01,                     // CMD_LO (PING)
        0x00,                     // LEN
        0x00,                     // CRC (placeholder, will calculate)
        TSUMIKORO_PACKET_END      // END
    };

    // Calculate correct CRC (over bytes 1-4: ID + CMD_HI + CMD_LO + LEN)
    buffer[5] = tsumikoro_crc8_calculate_table(&buffer[1], 4);

    tsumikoro_packet_t packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(0x01, packet.device_id);
    TEST_ASSERT_EQUAL(TSUMIKORO_CMD_PING, packet.command);
    TEST_ASSERT_EQUAL(0, packet.data_len);
}

/**
 * Test: Decode packet with data
 */
void test_decode_packet_with_data(void)
{
    // Manually constructed packet
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,  // START
        0x03,                     // ID
        0x00,                     // CMD_HI
        0x02,                     // CMD_LO (GET_INFO)
        0x03,                     // LEN
        0xAA,                     // DATA[0]
        0xBB,                     // DATA[1]
        0xCC,                     // DATA[2]
        0x00,                     // CRC (placeholder)
        TSUMIKORO_PACKET_END      // END
    };

    // Calculate correct CRC (over bytes 1-7: ID + CMD + LEN + DATA)
    buffer[8] = tsumikoro_crc8_calculate_table(&buffer[1], 7);

    tsumikoro_packet_t packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(0x03, packet.device_id);
    TEST_ASSERT_EQUAL(TSUMIKORO_CMD_GET_INFO, packet.command);
    TEST_ASSERT_EQUAL(3, packet.data_len);
    TEST_ASSERT_EQUAL(0xAA, packet.data[0]);
    TEST_ASSERT_EQUAL(0xBB, packet.data[1]);
    TEST_ASSERT_EQUAL(0xCC, packet.data[2]);
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
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, len, &decoded);

    // Verify
    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(original.device_id, decoded.device_id);
    TEST_ASSERT_EQUAL(original.command, decoded.command);
    TEST_ASSERT_EQUAL(original.data_len, decoded.data_len);

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
        TSUMIKORO_PACKET_START,
        0x01,
        0x00,
        0x01,
        0x00,
        0xFF,  // Incorrect CRC
        TSUMIKORO_PACKET_END
    };

    tsumikoro_packet_t packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

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
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_INVALID_PACKET, status);
}

/**
 * Test: Invalid END marker
 */
void test_decode_invalid_end(void)
{
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,
        0x01,
        0x00,
        0x01,
        0x00,
        0x00,
        0x00  // Wrong END marker
    };

    // Calculate correct CRC
    buffer[5] = tsumikoro_crc8_calculate_table(&buffer[1], 4);

    tsumikoro_packet_t packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_INVALID_PACKET, status);
}

/**
 * Test: Buffer too small
 */
void test_decode_buffer_too_small(void)
{
    uint8_t buffer[] = {
        TSUMIKORO_PACKET_START,
        0x01,
        0x00,
        0x01
        // Truncated packet
    };

    tsumikoro_packet_t packet;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, sizeof(buffer), &packet);

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

    // Should be 71 bytes
    TEST_ASSERT_EQUAL(TSUMIKORO_MAX_PACKET_LEN, len);

    // Verify decode
    tsumikoro_packet_t decoded;
    tsumikoro_status_t status = tsumikoro_packet_decode(buffer, len, &decoded);

    TEST_ASSERT_EQUAL(TSUMIKORO_STATUS_OK, status);
    TEST_ASSERT_EQUAL(TSUMIKORO_MAX_DATA_LEN, decoded.data_len);
    TEST_ASSERT_EQUAL_BYTES(packet.data, decoded.data, TSUMIKORO_MAX_DATA_LEN);
}

/**
 * Test: Inline helper functions
 */
void test_helper_functions(void)
{
    // Test packet size calculation
    TEST_ASSERT_EQUAL(7, tsumikoro_packet_size(0));
    TEST_ASSERT_EQUAL(8, tsumikoro_packet_size(1));
    TEST_ASSERT_EQUAL(71, tsumikoro_packet_size(64));

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
