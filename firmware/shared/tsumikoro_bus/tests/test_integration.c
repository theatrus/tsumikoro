/**
 * @file test_integration.c
 * @brief Integration tests for Tsumikoro bus protocol with mock HAL
 *
 * Tests controller-peripheral communication patterns using the mock HAL.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_framework.h"
#include "tsumikoro_protocol.h"
#include "tsumikoro_hal_mock.h"
#include <string.h>
#include <stdio.h>

/* ========== Test Fixtures ========== */

typedef struct {
    tsumikoro_packet_t last_packet;
    bool packet_received;
    size_t rx_count;
} test_device_context_t;

static void controller_rx_callback(const uint8_t *data, size_t len, void *user_data)
{
    test_device_context_t *ctx = (test_device_context_t *)user_data;

    tsumikoro_status_t status = tsumikoro_packet_decode(data, len, &ctx->last_packet);
    if (status == TSUMIKORO_STATUS_OK) {
        ctx->packet_received = true;
        ctx->rx_count++;
    }
}

static void peripheral_rx_callback(const uint8_t *data, size_t len, void *user_data)
{
    test_device_context_t *ctx = (test_device_context_t *)user_data;

    tsumikoro_status_t status = tsumikoro_packet_decode(data, len, &ctx->last_packet);
    if (status == TSUMIKORO_STATUS_OK) {
        ctx->packet_received = true;
        ctx->rx_count++;
    }
}

/* ========== Basic Communication Tests ========== */

static void test_controller_to_peripheral_ping(void)
{
    printf("  Testing controller to peripheral PING...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Create controller device
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral device
    test_device_context_t peripheral_ctx = {0};
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral = tsumikoro_mock_create_device(
        bus, &peripheral_config, peripheral_rx_callback, &peripheral_ctx);
    TEST_ASSERT(peripheral != NULL, "Failed to create peripheral device");

    // Send PING command from controller to peripheral
    tsumikoro_packet_t ping_packet = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&ping_packet, tx_buffer, sizeof(tx_buffer));
    TEST_ASSERT(tx_len > 0, "Failed to encode PING packet");

    tsumikoro_hal_status_t status = tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
    TEST_ASSERT(status == TSUMIKORO_HAL_OK, "Failed to transmit PING");

    // Process bus to deliver message
    tsumikoro_mock_bus_process(bus, 1);

    // Verify peripheral received PING
    TEST_ASSERT(peripheral_ctx.packet_received, "Peripheral did not receive PING");
    TEST_ASSERT(peripheral_ctx.last_packet.device_id == 0x01, "Wrong device ID in received packet");
    TEST_ASSERT(peripheral_ctx.last_packet.command == TSUMIKORO_CMD_PING, "Wrong command in received packet");
    TEST_ASSERT(peripheral_ctx.rx_count == 1, "Peripheral RX count should be 1");

    // Verify controller also received (RS-485 with RE asserted receives own transmission)
    // This is useful for collision detection
    TEST_ASSERT(controller_ctx.packet_received, "Controller did not receive own transmission");
    TEST_ASSERT(controller_ctx.rx_count == 1, "Controller RX count should be 1");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral);
    tsumikoro_mock_bus_destroy(bus);
}

static void test_peripheral_to_controller_response(void)
{
    printf("  Testing peripheral to controller response...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Create controller device
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral device
    test_device_context_t peripheral_ctx = {0};
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral = tsumikoro_mock_create_device(
        bus, &peripheral_config, peripheral_rx_callback, &peripheral_ctx);
    TEST_ASSERT(peripheral != NULL, "Failed to create peripheral device");

    // Send GET_STATUS from peripheral to controller
    tsumikoro_packet_t status_packet = {
        .device_id = 0x00,  // Addressed to controller
        .command = TSUMIKORO_CMD_GET_STATUS,
        .data_len = 8
    };
    memcpy(status_packet.data, "TESTDEV1", 8);

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&status_packet, tx_buffer, sizeof(tx_buffer));
    TEST_ASSERT(tx_len > 0, "Failed to encode STATUS packet");

    tsumikoro_hal_status_t status = tsumikoro_hal_transmit(peripheral, tx_buffer, tx_len);
    TEST_ASSERT(status == TSUMIKORO_HAL_OK, "Failed to transmit STATUS packet");

    // Process bus to deliver message
    tsumikoro_mock_bus_process(bus, 1);

    // Verify controller received packet
    TEST_ASSERT(controller_ctx.packet_received, "Controller did not receive packet");
    TEST_ASSERT(controller_ctx.last_packet.device_id == 0x00, "Wrong device ID in packet");
    TEST_ASSERT(controller_ctx.last_packet.command == TSUMIKORO_CMD_GET_STATUS,
                "Wrong command in packet");
    TEST_ASSERT(controller_ctx.last_packet.data_len == 8, "Wrong data length in packet");
    TEST_ASSERT(memcmp(controller_ctx.last_packet.data, "TESTDEV1", 8) == 0,
                "Wrong data in packet");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral);
    tsumikoro_mock_bus_destroy(bus);
}

static void test_multiple_peripherals(void)
{
    printf("  Testing multiple peripherals on bus...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Create controller
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral 1
    test_device_context_t peripheral1_ctx = {0};
    tsumikoro_hal_config_t peripheral1_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral1 = tsumikoro_mock_create_device(
        bus, &peripheral1_config, peripheral_rx_callback, &peripheral1_ctx);
    TEST_ASSERT(peripheral1 != NULL, "Failed to create peripheral 1");

    // Create peripheral 2
    test_device_context_t peripheral2_ctx = {0};
    tsumikoro_hal_config_t peripheral2_config = {
        .baud_rate = 1000000,
        .device_id = 0x02,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral2 = tsumikoro_mock_create_device(
        bus, &peripheral2_config, peripheral_rx_callback, &peripheral2_ctx);
    TEST_ASSERT(peripheral2 != NULL, "Failed to create peripheral 2");

    // Send command to peripheral 1
    tsumikoro_packet_t packet1 = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&packet1, tx_buffer, sizeof(tx_buffer));
    tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
    tsumikoro_mock_bus_process(bus, 1);

    // Verify peripheral 1 received
    TEST_ASSERT(peripheral1_ctx.packet_received, "Peripheral 1 did not receive packet");
    TEST_ASSERT(peripheral1_ctx.last_packet.device_id == 0x01, "Peripheral 1 wrong device ID");

    // Verify peripheral 2 also received (broadcast)
    TEST_ASSERT(peripheral2_ctx.packet_received, "Peripheral 2 did not receive packet");

    // Reset flags
    peripheral1_ctx.packet_received = false;
    peripheral2_ctx.packet_received = false;

    // Send command to peripheral 2
    tsumikoro_packet_t packet2 = {
        .device_id = 0x02,
        .command = TSUMIKORO_CMD_STOP,
        .data_len = 0
    };

    tx_len = tsumikoro_packet_encode(&packet2, tx_buffer, sizeof(tx_buffer));
    tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
    tsumikoro_mock_bus_process(bus, 1);

    // Verify peripheral 2 received
    TEST_ASSERT(peripheral2_ctx.packet_received, "Peripheral 2 did not receive second packet");
    TEST_ASSERT(peripheral2_ctx.last_packet.device_id == 0x02, "Peripheral 2 wrong device ID");
    TEST_ASSERT(peripheral2_ctx.last_packet.command == TSUMIKORO_CMD_STOP,
                "Peripheral 2 wrong command");

    // Verify peripheral 1 also received (broadcast)
    TEST_ASSERT(peripheral1_ctx.packet_received, "Peripheral 1 did not receive second packet");

    // Verify RX counts
    TEST_ASSERT(peripheral1_ctx.rx_count == 2, "Peripheral 1 should have received 2 packets");
    TEST_ASSERT(peripheral2_ctx.rx_count == 2, "Peripheral 2 should have received 2 packets");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral1);
    tsumikoro_hal_deinit(peripheral2);
    tsumikoro_mock_bus_destroy(bus);
}

/* ========== Error Injection Tests ========== */

static void test_crc_error_detection(void)
{
    printf("  Testing CRC error detection...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Create controller
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral
    test_device_context_t peripheral_ctx = {0};
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral = tsumikoro_mock_create_device(
        bus, &peripheral_config, peripheral_rx_callback, &peripheral_ctx);
    TEST_ASSERT(peripheral != NULL, "Failed to create peripheral device");

    // Inject CRC error
    tsumikoro_mock_bus_inject_error(bus, TSUMIKORO_MOCK_ERROR_CRC_CORRUPT);

    // Send packet
    tsumikoro_packet_t packet = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 4
    };
    packet.data[0] = 0x01;
    packet.data[1] = 0x02;
    packet.data[2] = 0x03;
    packet.data[3] = 0x04;

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&packet, tx_buffer, sizeof(tx_buffer));
    tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
    tsumikoro_mock_bus_process(bus, 1);

    // Verify peripheral did NOT receive packet (CRC check failed)
    TEST_ASSERT(!peripheral_ctx.packet_received, "Peripheral should reject corrupted packet");
    TEST_ASSERT(peripheral_ctx.rx_count == 0, "RX count should be 0 for corrupted packet");

    // Verify error was injected
    tsumikoro_mock_bus_stats_t stats;
    tsumikoro_mock_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.crc_errors_injected == 1, "CRC error should have been injected");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral);
    tsumikoro_mock_bus_destroy(bus);
}

static void test_packet_drop(void)
{
    printf("  Testing packet drop...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Create controller
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral
    test_device_context_t peripheral_ctx = {0};
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral = tsumikoro_mock_create_device(
        bus, &peripheral_config, peripheral_rx_callback, &peripheral_ctx);
    TEST_ASSERT(peripheral != NULL, "Failed to create peripheral device");

    // Inject packet drop error
    tsumikoro_mock_bus_inject_error(bus, TSUMIKORO_MOCK_ERROR_DROP_PACKET);

    // Send packet
    tsumikoro_packet_t packet = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&packet, tx_buffer, sizeof(tx_buffer));
    tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
    tsumikoro_mock_bus_process(bus, 1);

    // Verify no bytes pending on bus (packet was dropped)
    size_t pending = tsumikoro_mock_bus_get_pending_bytes(bus);
    TEST_ASSERT(pending == 0, "No bytes should be pending after drop");

    // Verify peripheral did not receive
    TEST_ASSERT(!peripheral_ctx.packet_received, "Peripheral should not receive dropped packet");

    // Verify drop was recorded
    tsumikoro_mock_bus_stats_t stats;
    tsumikoro_mock_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.packets_dropped == 1, "Packet drop should have been recorded");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral);
    tsumikoro_mock_bus_destroy(bus);
}

/* ========== Statistics Tests ========== */

static void test_bus_statistics(void)
{
    printf("  Testing bus statistics...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(bus != NULL, "Failed to create mock bus");

    // Reset stats
    tsumikoro_mock_bus_reset_stats(bus);

    // Create controller
    test_device_context_t controller_ctx = {0};
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller = tsumikoro_mock_create_device(
        bus, &controller_config, controller_rx_callback, &controller_ctx);
    TEST_ASSERT(controller != NULL, "Failed to create controller device");

    // Create peripheral
    test_device_context_t peripheral_ctx = {0};
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral = tsumikoro_mock_create_device(
        bus, &peripheral_config, peripheral_rx_callback, &peripheral_ctx);
    TEST_ASSERT(peripheral != NULL, "Failed to create peripheral device");

    // Send 3 packets
    tsumikoro_packet_t packet = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    uint8_t tx_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t tx_len = tsumikoro_packet_encode(&packet, tx_buffer, sizeof(tx_buffer));

    for (int i = 0; i < 3; i++) {
        tsumikoro_hal_transmit(controller, tx_buffer, tx_len);
        tsumikoro_mock_bus_process(bus, 1);
    }

    // Get stats
    tsumikoro_mock_bus_stats_t stats;
    tsumikoro_mock_bus_get_stats(bus, &stats);

    // Verify stats
    TEST_ASSERT(stats.packets_transmitted == 3, "Should have 3 packets transmitted");
    TEST_ASSERT(stats.packets_received == 6, "Should have 6 packets received (2 devices * 3 packets)");
    TEST_ASSERT(stats.packets_dropped == 0, "Should have 0 packets dropped");
    TEST_ASSERT(stats.crc_errors_injected == 0, "Should have 0 CRC errors");

    // Cleanup
    tsumikoro_hal_deinit(controller);
    tsumikoro_hal_deinit(peripheral);
    tsumikoro_mock_bus_destroy(bus);
}

/* ========== Main Test Runner ========== */

int main(void)
{
    printf("\n" COLOR_BOLD "Running Tsumikoro Bus Integration Tests" COLOR_RESET "\n");
    printf("==========================================\n\n");

    int assertions_run = 0;
    int assertions_passed = 0;

    // Save global counters
    extern int g_assertions_run;
    extern int g_assertions_passed;

    // Basic communication tests
    printf(COLOR_BOLD "Basic Communication Tests:\n" COLOR_RESET);
    test_controller_to_peripheral_ping();
    test_peripheral_to_controller_response();
    test_multiple_peripherals();
    printf("\n");

    // Error injection tests
    printf(COLOR_BOLD "Error Injection Tests:\n" COLOR_RESET);
    test_crc_error_detection();
    test_packet_drop();
    printf("\n");

    // Statistics tests
    printf(COLOR_BOLD "Statistics Tests:\n" COLOR_RESET);
    test_bus_statistics();
    printf("\n");

    // Get final counts
    assertions_run = g_assertions_run;
    assertions_passed = g_assertions_passed;

    // Print summary
    printf("==========================================\n");
    if (assertions_passed == assertions_run) {
        printf(COLOR_GREEN COLOR_BOLD "All tests passed! (%d/%d assertions)\n" COLOR_RESET,
               assertions_passed, assertions_run);
        return 0;
    } else {
        printf(COLOR_RED COLOR_BOLD "Some tests failed! (%d/%d assertions)\n" COLOR_RESET,
               assertions_passed, assertions_run);
        return 1;
    }
}
