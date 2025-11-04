/**
 * @file test_bus_handler.c
 * @brief Unit tests for bus handler (state machine, basic functionality)
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_framework.h"
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_mock.h"
#include <string.h>
#include <stdio.h>

/* ========== Basic Tests ========== */

static void test_bus_init_deinit(void)
{
    printf("  Testing bus init/deinit...\n");

    // Create mock bus and HAL
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();
    TEST_ASSERT(mock_bus != NULL, "Failed to create mock bus");

    tsumikoro_hal_config_t hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t hal = tsumikoro_mock_create_device(mock_bus, &hal_config, NULL, NULL);
    TEST_ASSERT(hal != NULL, "Failed to create HAL");

    // Create bus handler
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(hal, &bus_config, NULL, NULL);
    TEST_ASSERT(bus != NULL, "Failed to create bus handler");

    // Verify initial state
    TEST_ASSERT(tsumikoro_bus_get_state(bus) == TSUMIKORO_BUS_STATE_IDLE,
                "Initial state should be IDLE");
    TEST_ASSERT(tsumikoro_bus_is_idle(bus), "Bus should be idle");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

static void test_send_command_no_response(void)
{
    printf("  Testing send command without response...\n");

    // Create mock bus and HAL
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t hal = tsumikoro_mock_create_device(mock_bus, &hal_config, NULL, NULL);

    // Create bus handler
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(hal, &bus_config, NULL, NULL);

    // Send command without expecting response
    tsumikoro_packet_t cmd = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    tsumikoro_status_t status = tsumikoro_bus_send_no_response(bus, &cmd);
    TEST_ASSERT(status == TSUMIKORO_STATUS_OK, "Failed to send command");

    // Process bus - command should be sent immediately
    for (int i = 0; i < 10; i++) {
        tsumikoro_bus_process(bus);
        tsumikoro_mock_bus_process(mock_bus, 1);
    }

    // Verify bus returned to idle
    TEST_ASSERT(tsumikoro_bus_is_idle(bus), "Bus should return to idle");

    // Verify packet was transmitted
    tsumikoro_bus_stats_t stats;
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.commands_sent == 1, "Should have sent 1 command");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

static void test_statistics(void)
{
    printf("  Testing statistics tracking...\n");

    // Create mock bus and HAL
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();
    tsumikoro_hal_config_t hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t hal = tsumikoro_mock_create_device(mock_bus, &hal_config, NULL, NULL);

    // Create bus handler
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(hal, &bus_config, NULL, NULL);

    // Get initial stats
    tsumikoro_bus_stats_t stats;
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.commands_sent == 0, "Initial commands_sent should be 0");

    // Send a command
    tsumikoro_packet_t cmd = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };
    tsumikoro_bus_send_no_response(bus, &cmd);

    for (int i = 0; i < 10; i++) {
        tsumikoro_bus_process(bus);
        tsumikoro_mock_bus_process(mock_bus, 1);
    }

    // Check stats
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.commands_sent == 1, "Should have sent 1 command");

    // Reset stats
    tsumikoro_bus_reset_stats(bus);
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.commands_sent == 0, "Stats should be reset");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

typedef struct {
    tsumikoro_packet_t last_unsolicited;
    bool unsolicited_received;
    int unsolicited_count;
} test_unsolicited_context_t;

static void test_unsolicited_callback(const tsumikoro_packet_t *packet,
                                       void *user_data)
{
    test_unsolicited_context_t *ctx = (test_unsolicited_context_t *)user_data;
    if (packet) {
        ctx->last_unsolicited = *packet;
    }
    ctx->unsolicited_received = true;
    ctx->unsolicited_count++;
}

static void test_unsolicited_messages(void)
{
    printf("  Testing unsolicited message handling...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();

    // Create controller HAL
    tsumikoro_hal_config_t controller_hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller_hal = tsumikoro_mock_create_device(
        mock_bus, &controller_hal_config, NULL, NULL);

    // Create peripheral HAL
    tsumikoro_hal_config_t peripheral_hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t peripheral_hal = tsumikoro_mock_create_device(
        mock_bus, &peripheral_hal_config, NULL, NULL);

    // Create bus handler with unsolicited callback
    test_unsolicited_context_t unsolicited_ctx = {0};
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(controller_hal, &bus_config,
                                                     test_unsolicited_callback, &unsolicited_ctx);

    // Send unsolicited message from peripheral (when no command is pending)
    tsumikoro_packet_t unsolicited_msg = {
        .device_id = 0x00,
        .command = TSUMIKORO_CMD_GET_STATUS,
        .data_len = 2
    };
    unsolicited_msg.data[0] = 0xAB;
    unsolicited_msg.data[1] = 0xCD;

    uint8_t msg_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t msg_len = tsumikoro_packet_encode(&unsolicited_msg, msg_buffer, sizeof(msg_buffer));
    tsumikoro_hal_transmit(peripheral_hal, msg_buffer, msg_len);

    // Process to receive unsolicited message
    for (int i = 0; i < 10; i++) {
        tsumikoro_mock_bus_process(mock_bus, 1);
        tsumikoro_bus_process(bus);
    }

    // Verify unsolicited callback was invoked
    TEST_ASSERT(unsolicited_ctx.unsolicited_received, "Unsolicited callback should be invoked");
    TEST_ASSERT(unsolicited_ctx.last_unsolicited.command == TSUMIKORO_CMD_GET_STATUS,
                "Unsolicited message command should match");
    TEST_ASSERT(unsolicited_ctx.last_unsolicited.data_len == 2,
                "Unsolicited message data length should be 2");

    // Verify statistics
    tsumikoro_bus_stats_t stats;
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.unsolicited_messages == 1, "Should have 1 unsolicited message");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(controller_hal);
    tsumikoro_hal_deinit(peripheral_hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

/* ========== Main Test Runner ========== */

int main(void)
{
    printf("\n" COLOR_BOLD "Running Tsumikoro Bus Handler Tests" COLOR_RESET "\n");
    printf("==========================================\n\n");

    // Basic tests
    printf(COLOR_BOLD "Basic Bus Handler Tests:\n" COLOR_RESET);
    test_bus_init_deinit();
    test_send_command_no_response();
    test_statistics();
    test_unsolicited_messages();
    printf("\n");

    // Print summary
    printf("==========================================\n");
    if (g_assertions_passed == g_assertions_run) {
        printf(COLOR_GREEN COLOR_BOLD "All tests passed! (%d/%d assertions)\n" COLOR_RESET,
               g_assertions_passed, g_assertions_run);
        return 0;
    } else {
        printf(COLOR_RED COLOR_BOLD "Some tests failed! (%d/%d assertions)\n" COLOR_RESET,
               g_assertions_passed, g_assertions_run);
        return 1;
    }
}
