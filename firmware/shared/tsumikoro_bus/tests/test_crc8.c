/**
 * @file test_crc8.c
 * @brief Unit tests for CRC8-CCITT implementation
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_framework.h"
#include "tsumikoro_crc8.h"

/**
 * Test: Empty data should return 0
 */
void test_crc8_empty_data(void)
{
    uint8_t crc = tsumikoro_crc8_calculate(NULL, 0);
    TEST_ASSERT_EQUAL(0, crc);

    crc = tsumikoro_crc8_calculate_table(NULL, 0);
    TEST_ASSERT_EQUAL(0, crc);
}

/**
 * Test: Single byte CRC
 */
void test_crc8_single_byte(void)
{
    uint8_t data[] = { 0x00 };
    uint8_t crc_bitwise = tsumikoro_crc8_calculate(data, 1);
    uint8_t crc_table = tsumikoro_crc8_calculate_table(data, 1);

    // Both implementations should match
    TEST_ASSERT_EQUAL(crc_bitwise, crc_table);

    // CRC of 0x00 should be 0x00 (polynomial 0x07, init 0x00)
    TEST_ASSERT_EQUAL(0x00, crc_bitwise);
}

/**
 * Test: Known test vectors
 */
void test_crc8_known_vectors(void)
{
    // Test vector 1: Simple sequence
    uint8_t data1[] = { 0x01, 0x02, 0x03 };
    uint8_t crc1_bitwise = tsumikoro_crc8_calculate(data1, sizeof(data1));
    uint8_t crc1_table = tsumikoro_crc8_calculate_table(data1, sizeof(data1));
    TEST_ASSERT_EQUAL(crc1_bitwise, crc1_table);

    // Test vector 2: ASCII "123456789"
    uint8_t data2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint8_t crc2_bitwise = tsumikoro_crc8_calculate(data2, sizeof(data2));
    uint8_t crc2_table = tsumikoro_crc8_calculate_table(data2, sizeof(data2));
    TEST_ASSERT_EQUAL(crc2_bitwise, crc2_table);

    // Known CRC8-CCITT of "123456789" is 0xF4
    TEST_ASSERT_EQUAL(0xF4, crc2_bitwise);

    // Test vector 3: All 0xFF
    uint8_t data3[] = { 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t crc3_bitwise = tsumikoro_crc8_calculate(data3, sizeof(data3));
    uint8_t crc3_table = tsumikoro_crc8_calculate_table(data3, sizeof(data3));
    TEST_ASSERT_EQUAL(crc3_bitwise, crc3_table);
}

/**
 * Test: Packet-like data
 */
void test_crc8_packet_data(void)
{
    // Simulate packet: ID=0x01, CMD=0x0001 (PING), LEN=0
    uint8_t packet[] = { 0x01, 0x00, 0x01, 0x00 };
    uint8_t crc_bitwise = tsumikoro_crc8_calculate(packet, sizeof(packet));
    uint8_t crc_table = tsumikoro_crc8_calculate_table(packet, sizeof(packet));

    TEST_ASSERT_EQUAL(crc_bitwise, crc_table);
    printf("    CRC of packet [01 00 01 00]: 0x%02X\n", crc_bitwise);
}

/**
 * Test: CRC verification function
 */
void test_crc8_verify(void)
{
    uint8_t data[] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t correct_crc = tsumikoro_crc8_calculate_table(data, sizeof(data));

    // Should pass with correct CRC
    TEST_ASSERT(tsumikoro_crc8_verify(data, sizeof(data), correct_crc),
                "Verification should pass with correct CRC");

    // Should fail with incorrect CRC
    TEST_ASSERT(!tsumikoro_crc8_verify(data, sizeof(data), correct_crc + 1),
                "Verification should fail with incorrect CRC");
}

/**
 * Test: Large data block
 */
void test_crc8_large_data(void)
{
    uint8_t data[64];
    for (int i = 0; i < 64; i++) {
        data[i] = (uint8_t)i;
    }

    uint8_t crc_bitwise = tsumikoro_crc8_calculate(data, sizeof(data));
    uint8_t crc_table = tsumikoro_crc8_calculate_table(data, sizeof(data));

    TEST_ASSERT_EQUAL(crc_bitwise, crc_table);
    printf("    CRC of 64-byte sequence: 0x%02X\n", crc_bitwise);
}

/**
 * Test: Bit error detection
 */
void test_crc8_error_detection(void)
{
    uint8_t data[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
    uint8_t crc_original = tsumikoro_crc8_calculate_table(data, sizeof(data));

    // Flip single bit in data[2]
    data[2] ^= 0x01;
    uint8_t crc_corrupted = tsumikoro_crc8_calculate_table(data, sizeof(data));

    // CRC should change
    TEST_ASSERT(crc_original != crc_corrupted,
                "CRC should detect single-bit error");

    // Restore and flip two bits
    data[2] ^= 0x01;  // Restore
    data[1] ^= 0x01;  // Flip bit in data[1]
    data[3] ^= 0x80;  // Flip bit in data[3]
    uint8_t crc_double_error = tsumikoro_crc8_calculate_table(data, sizeof(data));

    // CRC should detect double-bit error
    TEST_ASSERT(crc_original != crc_double_error,
                "CRC should detect double-bit error");
}

/**
 * Main test runner
 */
int main(void)
{
    TEST_SUITE("CRC8-CCITT Tests");

    RUN_TEST(test_crc8_empty_data);
    RUN_TEST(test_crc8_single_byte);
    RUN_TEST(test_crc8_known_vectors);
    RUN_TEST(test_crc8_packet_data);
    RUN_TEST(test_crc8_verify);
    RUN_TEST(test_crc8_large_data);
    RUN_TEST(test_crc8_error_detection);

    TEST_SUMMARY();
    return TEST_EXIT();
}
