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

/* ========== Test Helpers ========== */

/**
 * @brief Auto-responder context for peripheral devices
 */
typedef struct {
    tsumikoro_mock_bus_handle_t bus;
    tsumikoro_hal_handle_t hal;
    uint8_t device_id;
} auto_responder_t;

/**
 * @brief Auto-responder RX callback - automatically responds to commands
 */
static void auto_responder_callback(const uint8_t *data, size_t len, void *user_data)
{
    auto_responder_t *responder = (auto_responder_t *)user_data;

    // Decode received packet
    tsumikoro_packet_t rx_packet;
    size_t bytes_consumed = 0;
    if (tsumikoro_packet_decode(data, len, &rx_packet, &bytes_consumed) != TSUMIKORO_STATUS_OK) {
        printf("    [AUTO-RESPONDER] Decode failed\n");
        return;
    }

    // Only respond if addressed to us
    if (rx_packet.device_id != responder->device_id) {
        printf("    [AUTO-RESPONDER] Ignoring packet for device 0x%02X (we are 0x%02X)\n",
               rx_packet.device_id, responder->device_id);
        return;
    }

    printf("    [AUTO-RESPONDER] Received command 0x%04X for device 0x%02X, sending response\n",
           rx_packet.command, rx_packet.device_id);

    // Send automatic response
    tsumikoro_packet_t response = {
        .device_id = 0x00,  // Address response to controller
        .command = rx_packet.command,  // Echo command
        .data_len = 4
    };
    response.data[0] = 0xAA;
    response.data[1] = 0xBB;
    response.data[2] = 0xCC;
    response.data[3] = 0xDD;

    uint8_t resp_buffer[TSUMIKORO_MAX_PACKET_LEN];
    size_t resp_len = tsumikoro_packet_encode(&response, resp_buffer, sizeof(resp_buffer));
    printf("    [AUTO-RESPONDER] Encoded response: len=%zu, data_len=%u, data[0]=0x%02X\n",
           resp_len, response.data_len, response.data[0]);
    if (resp_len > 0) {
        tsumikoro_hal_status_t tx_status = tsumikoro_hal_transmit(responder->hal, resp_buffer, resp_len);
        printf("    [AUTO-RESPONDER] Transmit status: %d (%s)\n", tx_status,
               tx_status == TSUMIKORO_HAL_OK ? "OK" :
               tx_status == TSUMIKORO_HAL_BUSY ? "BUSY" : "ERROR");
        // Don't call tsumikoro_mock_bus_process() here - that would be recursive!
        // The test code will call it to deliver this response
    }
}

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
        .turnaround_delay_bytes = 0,
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
        .turnaround_delay_bytes = 0,
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
        .turnaround_delay_bytes = 0,
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
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller_hal = tsumikoro_mock_create_device(
        mock_bus, &controller_hal_config, NULL, NULL);

    // Create peripheral HAL
    tsumikoro_hal_config_t peripheral_hal_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .turnaround_delay_bytes = 0,
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

/* ========== Advanced Tests ========== */

static void test_blocking_send_with_response(void)
{
    printf("  Testing blocking send with response...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();

    // Create controller HAL
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller_hal = tsumikoro_mock_create_device(
        mock_bus, &controller_config, NULL, NULL);

    // Create auto-responder peripheral
    auto_responder_t responder = {
        .bus = mock_bus,
        .device_id = 0x01
    };
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    responder.hal = tsumikoro_mock_create_device(
        mock_bus, &peripheral_config, auto_responder_callback, &responder);

    // Create bus handler
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(controller_hal, &bus_config, NULL, NULL);

    // Send blocking command
    tsumikoro_packet_t cmd = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_GET_STATUS,
        .data_len = 0
    };

    tsumikoro_packet_t response;
    tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
        bus, &cmd, &response, 200);  // 200ms timeout

    // Verify success
    TEST_ASSERT(status == TSUMIKORO_CMD_STATUS_SUCCESS, "Blocking send should succeed");
    TEST_ASSERT(response.command == TSUMIKORO_CMD_GET_STATUS, "Response command should match");
    TEST_ASSERT(response.data_len == 4, "Response should have 4 bytes of data");
    TEST_ASSERT(response.data[0] == 0xAA, "Response data should match");

    // Verify bus is idle after blocking send
    TEST_ASSERT(tsumikoro_bus_is_idle(bus), "Bus should be idle after blocking send");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(controller_hal);
    tsumikoro_hal_deinit(responder.hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

static void test_blocking_send_timeout(void)
{
    printf("  Testing blocking send timeout...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();

    // Create controller HAL
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller_hal = tsumikoro_mock_create_device(
        mock_bus, &controller_config, NULL, NULL);

    // Create bus handler with short timeout for testing
    tsumikoro_bus_config_t bus_config = {
        .response_timeout_ms = 10,
        .retry_count = 1,
        .retry_delay_ms = 5,
        .bus_idle_timeout_ms = 5,
        .auto_retry = true
    };
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(controller_hal, &bus_config, NULL, NULL);

    // Send blocking command (no peripheral to respond)
    tsumikoro_packet_t cmd = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_PING,
        .data_len = 0
    };

    tsumikoro_packet_t response;
    tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
        bus, &cmd, &response, 50);  // 50ms total timeout

    // Verify timeout
    TEST_ASSERT(status == TSUMIKORO_CMD_STATUS_TIMEOUT, "Should timeout without response");

    // Verify retries occurred
    tsumikoro_bus_stats_t stats;
    tsumikoro_bus_get_stats(bus, &stats);
    TEST_ASSERT(stats.retries >= 1, "Should have retried at least once");
    TEST_ASSERT(stats.timeouts == 1, "Should have 1 timeout");

    // Verify bus is idle after timeout
    TEST_ASSERT(tsumikoro_bus_is_idle(bus), "Bus should be idle after timeout");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(controller_hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

typedef struct {
    tsumikoro_cmd_status_t last_status;
    tsumikoro_packet_t last_response;
    bool callback_invoked;
} async_test_context_t;

static void async_test_callback(tsumikoro_cmd_status_t status,
                                 const tsumikoro_packet_t *response,
                                 void *user_data)
{
    async_test_context_t *ctx = (async_test_context_t *)user_data;
    ctx->last_status = status;
    if (response) {
        printf("    [CALLBACK] Received response: cmd=0x%04X, data_len=%u, data[0]=0x%02X\n",
               response->command, response->data_len, response->data_len > 0 ? response->data[0] : 0);
        ctx->last_response = *response;
    } else {
        printf("    [CALLBACK] No response (status=%d)\n", status);
    }
    ctx->callback_invoked = true;
}

static void test_async_send_with_response(void)
{
    printf("  Testing async send with response...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();

    // Create controller HAL
    tsumikoro_hal_config_t controller_config = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t controller_hal = tsumikoro_mock_create_device(
        mock_bus, &controller_config, NULL, NULL);

    // Create auto-responder peripheral
    auto_responder_t responder = {
        .bus = mock_bus,
        .device_id = 0x01
    };
    tsumikoro_hal_config_t peripheral_config = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    responder.hal = tsumikoro_mock_create_device(
        mock_bus, &peripheral_config, auto_responder_callback, &responder);

    // Create bus handler
    tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(controller_hal, &bus_config, NULL, NULL);

    // Send async command
    async_test_context_t ctx = {0};
    tsumikoro_packet_t cmd = {
        .device_id = 0x01,
        .command = TSUMIKORO_CMD_GET_VERSION,
        .data_len = 0
    };

    tsumikoro_status_t send_status = tsumikoro_bus_send_command_async(
        bus, &cmd, async_test_callback, &ctx);
    TEST_ASSERT(send_status == TSUMIKORO_STATUS_OK, "Async send should succeed");

    // Process bus until callback is invoked
    for (int i = 0; i < 200 && !ctx.callback_invoked; i++) {
        tsumikoro_bus_process(bus);
        tsumikoro_mock_bus_process(mock_bus, 1);
    }

    // Verify callback was invoked with success
    TEST_ASSERT(ctx.callback_invoked, "Callback should be invoked");
    TEST_ASSERT(ctx.last_status == TSUMIKORO_CMD_STATUS_SUCCESS, "Status should be SUCCESS");
    TEST_ASSERT(ctx.last_response.command == TSUMIKORO_CMD_GET_VERSION, "Response command should match");
    TEST_ASSERT(ctx.last_response.data_len == 4, "Response should have data");

    // Cleanup
    tsumikoro_bus_deinit(bus);
    tsumikoro_hal_deinit(controller_hal);
    tsumikoro_hal_deinit(responder.hal);
    tsumikoro_mock_bus_destroy(mock_bus);
}

static void test_turnaround_delay(void)
{
    printf("  Testing bus turnaround delay...\n");

    // Create mock bus
    tsumikoro_mock_bus_handle_t mock_bus = tsumikoro_mock_bus_create();

    // Test 1: With turnaround delay of 3 bytes at 1Mbaud
    // At 1Mbaud with 10 bits/byte, 1 byte = 10us, so 3 bytes = 30us ~= 1ms (rounded up)
    tsumikoro_hal_config_t hal_config_with_delay = {
        .baud_rate = 1000000,
        .device_id = 0x00,
        .is_controller = true,
        .turnaround_delay_bytes = 3,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t hal_with_delay = tsumikoro_mock_create_device(
        mock_bus, &hal_config_with_delay, NULL, NULL);

    // Initially, bus should be idle (no activity yet)
    TEST_ASSERT(tsumikoro_hal_is_bus_idle(hal_with_delay),
                "Bus should be idle before any activity");

    // Transmit a packet
    uint8_t test_data[] = {0xAA, 0x01, 0x00, 0x05, 0x00, 0xBB, 0x55};
    tsumikoro_hal_transmit(hal_with_delay, test_data, sizeof(test_data));

    // Process the transmission
    tsumikoro_mock_bus_process(mock_bus, 0);

    // Immediately after transmission, bus should NOT be idle (turnaround delay not elapsed)
    bool idle_immediately = tsumikoro_hal_is_bus_idle(hal_with_delay);
    TEST_ASSERT(!idle_immediately,
                "Bus should NOT be idle immediately after transmission");

    // Advance time by less than turnaround delay
    tsumikoro_mock_bus_process(mock_bus, 0);  // Still at same time
    TEST_ASSERT(!tsumikoro_hal_is_bus_idle(hal_with_delay),
                "Bus should still not be idle before turnaround delay");

    // Advance time past turnaround delay (need at least 1ms)
    tsumikoro_mock_bus_process(mock_bus, 1);
    TEST_ASSERT(tsumikoro_hal_is_bus_idle(hal_with_delay),
                "Bus should be idle after turnaround delay elapsed");

    // Test 2: With no turnaround delay (0 bytes)
    tsumikoro_hal_config_t hal_config_no_delay = {
        .baud_rate = 1000000,
        .device_id = 0x01,
        .is_controller = false,
        .turnaround_delay_bytes = 0,
        .platform_data = NULL
    };
    tsumikoro_hal_handle_t hal_no_delay = tsumikoro_mock_create_device(
        mock_bus, &hal_config_no_delay, NULL, NULL);

    // Transmit a packet
    tsumikoro_hal_transmit(hal_no_delay, test_data, sizeof(test_data));

    // Process the transmission
    tsumikoro_mock_bus_process(mock_bus, 0);

    // With no turnaround delay, bus should be idle immediately
    TEST_ASSERT(tsumikoro_hal_is_bus_idle(hal_no_delay),
                "Bus should be idle immediately with turnaround_delay_bytes=0");

    // Cleanup
    tsumikoro_hal_deinit(hal_with_delay);
    tsumikoro_hal_deinit(hal_no_delay);
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
    test_turnaround_delay();
    printf("\n");

    // Advanced tests with auto-responder
    printf(COLOR_BOLD "Advanced Bus Handler Tests:\n" COLOR_RESET);
    test_async_send_with_response();
    test_blocking_send_with_response();
    test_blocking_send_timeout();
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
