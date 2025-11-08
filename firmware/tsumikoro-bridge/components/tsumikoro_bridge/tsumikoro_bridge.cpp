#include "tsumikoro_bridge.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace tsumikoro_bridge {

static const char *const TAG = "tsumikoro_bridge";

void TsumikoroBridge::setup() {
  ESP_LOGI(TAG, "Setting up Tsumikoro Bridge (Bus Controller)...");

  // Configure ESP32 HAL
  this->esp32_config_.uart_port = static_cast<uart_port_t>(this->uart_port_);
  this->esp32_config_.tx_pin = this->tx_pin_;
  this->esp32_config_.rx_pin = this->rx_pin_;
  this->esp32_config_.rts_pin = this->de_pin_;
  this->esp32_config_.cts_pin = -1;  // Not used
  this->esp32_config_.rx_buffer_size = 1024;
  this->esp32_config_.tx_buffer_size = 1024;
  this->esp32_config_.use_rs485_mode = (this->de_pin_ >= 0);

  // Initialize UART peripheral
  if (!tsumikoro_hal_esp32_init_peripheral(&this->esp32_config_, this->baud_rate_)) {
    ESP_LOGE(TAG, "Failed to initialize UART peripheral");
    this->mark_failed();
    return;
  }

  // Initialize HAL
  tsumikoro_hal_config_t hal_config = {
    .baud_rate = this->baud_rate_,
    .device_id = this->device_id_,
    .is_controller = true,  // This is a controller
    .turnaround_delay_bytes = this->turnaround_delay_,
    .platform_data = (void *)&this->esp32_config_
  };

  ESP_LOGE(TAG, "=== ABOUT TO CALL tsumikoro_hal_init ===");
  this->hal_handle_ = tsumikoro_hal_init(&hal_config, nullptr, nullptr);
  ESP_LOGE(TAG, "=== RETURNED FROM tsumikoro_hal_init, handle=%p ===", this->hal_handle_);

  if (!this->hal_handle_) {
    ESP_LOGE(TAG, "Failed to initialize HAL");
    this->mark_failed();
    return;
  }

  ESP_LOGE(TAG, "=== HAL INITIALIZED SUCCESSFULLY ===");

  // Initialize bus handler (RTOS mode)
  tsumikoro_bus_config_t bus_config = TSUMIKORO_BUS_DEFAULT_CONFIG();
  bus_config.response_timeout_ms = 100;
  bus_config.retry_count = 3;
  bus_config.retry_delay_ms = 10;

  this->bus_handle_ = tsumikoro_bus_init(
    this->hal_handle_,
    &bus_config,
    &TsumikoroBridge::bus_unsolicited_callback_,
    this  // user_data = this
  );

  if (!this->bus_handle_) {
    ESP_LOGE(TAG, "Failed to initialize bus handler");
    this->mark_failed();
    return;
  }

#if TSUMIKORO_BUS_USE_RTOS
  // In RTOS mode, re-initialize HAL with RX callback
  tsumikoro_hal_deinit(this->hal_handle_);
  tsumikoro_hal_rx_callback_t rx_callback = tsumikoro_bus_get_hal_rx_callback();
  this->hal_handle_ = tsumikoro_hal_init(&hal_config, rx_callback, this->bus_handle_);
  if (!this->hal_handle_) {
    ESP_LOGE(TAG, "Failed to re-initialize HAL with RX callback");
    this->mark_failed();
    return;
  }
#endif

  ESP_LOGI(TAG, "Tsumikoro Bridge setup complete");
  ESP_LOGI(TAG, "  Device ID: 0x%02X (Controller)", this->device_id_);
  ESP_LOGI(TAG, "  UART%d: %d baud, TX=%d, RX=%d, DE=%d",
           this->uart_port_, this->baud_rate_,
           this->tx_pin_, this->rx_pin_, this->de_pin_);
  ESP_LOGI(TAG, "  RS-485 mode: %s", this->esp32_config_.use_rs485_mode ? "enabled" : "disabled");
  ESP_LOGI(TAG, "  Turnaround delay: %d byte intervals", this->turnaround_delay_);
}

void TsumikoroBridge::loop() {
  // In RTOS mode, the bus protocol runs in threads
  // This loop is minimal - just for statistics reporting
#if !TSUMIKORO_BUS_USE_RTOS
  // In bare-metal mode, we'd need to call tsumikoro_bus_process()
  // But we're using RTOS mode, so this isn't needed
#endif
}

void TsumikoroBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "Tsumikoro Bridge:");
  ESP_LOGCONFIG(TAG, "  Device ID: 0x%02X (Controller)", this->device_id_);
  ESP_LOGCONFIG(TAG, "  UART Port: %d", this->uart_port_);
  ESP_LOGCONFIG(TAG, "  Baud Rate: %d", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "  TX Pin: GPIO%d", this->tx_pin_);
  ESP_LOGCONFIG(TAG, "  RX Pin: GPIO%d", this->rx_pin_);
  if (this->de_pin_ >= 0) {
    ESP_LOGCONFIG(TAG, "  DE Pin: GPIO%d", this->de_pin_);
  }
  ESP_LOGCONFIG(TAG, "  Turnaround Delay: %d byte intervals", this->turnaround_delay_);

  // Print bus statistics
  if (this->bus_handle_) {
    tsumikoro_bus_stats_t stats;
    tsumikoro_bus_get_stats(this->bus_handle_, &stats);
    ESP_LOGCONFIG(TAG, "  Bus Statistics:");
    ESP_LOGCONFIG(TAG, "    Commands sent: %d", stats.commands_sent);
    ESP_LOGCONFIG(TAG, "    Responses received: %d", stats.responses_received);
    ESP_LOGCONFIG(TAG, "    Timeouts: %d", stats.timeouts);
    ESP_LOGCONFIG(TAG, "    Retries: %d", stats.retries);
    ESP_LOGCONFIG(TAG, "    CRC errors: %d", stats.crc_errors);
  }
}

bool TsumikoroBridge::ping_device(uint8_t device_id, uint32_t timeout_ms) {
  if (!this->bus_handle_) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_PING,
    .data_len = 0
  };

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, timeout_ms
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS) {
    ESP_LOGD(TAG, "PING device 0x%02X: success", device_id);
    return true;
  } else {
    ESP_LOGW(TAG, "PING device 0x%02X: failed (status=%d)", device_id, status);
    return false;
  }
}

bool TsumikoroBridge::get_version(uint8_t device_id, uint8_t *major, uint8_t *minor, uint8_t *patch) {
  if (!this->bus_handle_ || !major || !minor || !patch) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_GET_VERSION,
    .data_len = 0
  };

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS && response.data_len >= 3) {
    *major = response.data[0];
    *minor = response.data[1];
    *patch = response.data[2];
    ESP_LOGD(TAG, "GET_VERSION device 0x%02X: v%d.%d.%d", device_id, *major, *minor, *patch);
    return true;
  } else {
    ESP_LOGW(TAG, "GET_VERSION device 0x%02X: failed", device_id);
    return false;
  }
}

bool TsumikoroBridge::get_status(uint8_t device_id, uint8_t *status, size_t *status_len) {
  if (!this->bus_handle_ || !status || !status_len) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_GET_STATUS,
    .data_len = 0
  };

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t result = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (result == TSUMIKORO_CMD_STATUS_SUCCESS) {
    size_t copy_len = (response.data_len < *status_len) ? response.data_len : *status_len;
    memcpy(status, response.data, copy_len);
    *status_len = copy_len;
    ESP_LOGD(TAG, "GET_STATUS device 0x%02X: %d bytes", device_id, copy_len);
    return true;
  } else {
    ESP_LOGW(TAG, "GET_STATUS device 0x%02X: failed", device_id);
    return false;
  }
}

bool TsumikoroBridge::stepper_move(uint8_t device_id, int32_t position) {
  if (!this->bus_handle_) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_STEPPER_MOVE,
    .data_len = 4
  };

  // Pack position as big-endian int32_t
  cmd.data[0] = (position >> 24) & 0xFF;
  cmd.data[1] = (position >> 16) & 0xFF;
  cmd.data[2] = (position >> 8) & 0xFF;
  cmd.data[3] = position & 0xFF;

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "STEPPER_MOVE device 0x%02X to position %d: success", device_id, position);
    return true;
  } else {
    ESP_LOGW(TAG, "STEPPER_MOVE device 0x%02X: failed", device_id);
    return false;
  }
}

bool TsumikoroBridge::stepper_stop(uint8_t device_id) {
  if (!this->bus_handle_) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_STEPPER_STOP,
    .data_len = 0
  };

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "STEPPER_STOP device 0x%02X: success", device_id);
    return true;
  } else {
    ESP_LOGW(TAG, "STEPPER_STOP device 0x%02X: failed", device_id);
    return false;
  }
}

bool TsumikoroBridge::servo_set_position(uint8_t device_id, uint16_t position) {
  if (!this->bus_handle_) {
    return false;
  }

  // Clamp position to valid range (0-1800)
  if (position > 1800) {
    position = 1800;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_SERVO_SET_POSITION,
    .data_len = 2
  };

  // Pack position as big-endian uint16_t
  cmd.data[0] = (position >> 8) & 0xFF;
  cmd.data[1] = position & 0xFF;

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS) {
    ESP_LOGI(TAG, "SERVO_SET_POSITION device 0x%02X to %d (%.1f degrees): success",
             device_id, position, position / 10.0f);
    return true;
  } else {
    ESP_LOGW(TAG, "SERVO_SET_POSITION device 0x%02X: failed", device_id);
    return false;
  }
}

bool TsumikoroBridge::servo_get_position(uint8_t device_id, uint16_t *position) {
  if (!this->bus_handle_ || !position) {
    return false;
  }

  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = TSUMIKORO_CMD_SERVO_GET_POSITION,
    .data_len = 0
  };

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS && response.data_len >= 2) {
    *position = (response.data[0] << 8) | response.data[1];
    ESP_LOGD(TAG, "SERVO_GET_POSITION device 0x%02X: %d (%.1f degrees)",
             device_id, *position, *position / 10.0f);
    return true;
  } else {
    ESP_LOGW(TAG, "SERVO_GET_POSITION device 0x%02X: failed", device_id);
    return false;
  }
}

tsumikoro_bus_stats_t TsumikoroBridge::get_bus_stats() const {
  tsumikoro_bus_stats_t stats = {0};

  if (this->bus_handle_) {
    tsumikoro_bus_get_stats(this->bus_handle_, &stats);
  }

  return stats;
}

void TsumikoroBridge::bus_unsolicited_callback_(const tsumikoro_packet_t *packet, void *user_data) {
  TsumikoroBridge *instance = static_cast<TsumikoroBridge *>(user_data);
  if (instance && packet) {
    instance->handle_unsolicited_message_(packet);
  }
}

void TsumikoroBridge::handle_unsolicited_message_(const tsumikoro_packet_t *packet) {
  // Handle unsolicited messages from peripherals
  ESP_LOGD(TAG, "Unsolicited message from device 0x%02X, command 0x%04X, %d bytes",
           packet->device_id, packet->command, packet->data_len);

  // TODO: Publish to Home Assistant as sensor updates, events, etc.
  // For example:
  // - Stepper position updates
  // - Servo position updates
  // - Error conditions
  // - Status changes
}

bool TsumikoroBridge::nucleo_set_led(uint8_t device_id, uint8_t state) {
  if (!this->bus_handle_) {
    return false;
  }

  // Custom command 0xF001: Set LED state
  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = 0xF001,
    .data_len = 1
  };
  cmd.data[0] = state;

  tsumikoro_packet_t response;
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS && response.data_len >= 1) {
    bool success = (response.data[0] == 0x00);
    if (success) {
      const char *state_str = (state == 0) ? "OFF" : (state == 1) ? "ON" : "AUTO-BLINK";
      ESP_LOGI(TAG, "NUCLEO_SET_LED device 0x%02X to %s: success", device_id, state_str);
    } else {
      ESP_LOGW(TAG, "NUCLEO_SET_LED device 0x%02X: device returned error", device_id);
    }
    return success;
  } else {
    ESP_LOGW(TAG, "NUCLEO_SET_LED device 0x%02X: failed (status=%d)", device_id, status);
    return false;
  }
}

bool TsumikoroBridge::nucleo_get_led(uint8_t device_id, uint8_t *led_state, uint8_t *auto_blink) {
  if (!this->bus_handle_ || !led_state || !auto_blink) {
    return false;
  }

  // Custom command 0xF002: Get LED state
  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = 0xF002,
    .data_len = 0
  };

  tsumikoro_packet_t response;

  ESP_LOGI(TAG, ">>> Calling tsumikoro_bus_send_command_blocking for GET_LED, device=0x%02X", device_id);
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );
  ESP_LOGI(TAG, "<<< tsumikoro_bus_send_command_blocking returned status=%d", status);

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS && response.data_len >= 2) {
    *led_state = response.data[0];
    *auto_blink = response.data[1];
    ESP_LOGD(TAG, "NUCLEO_GET_LED device 0x%02X: LED=%s, auto-blink=%s",
             device_id,
             *led_state ? "ON" : "OFF",
             *auto_blink ? "enabled" : "disabled");
    return true;
  } else {
    ESP_LOGW(TAG, "NUCLEO_GET_LED device 0x%02X: failed (status=%d, data_len=%u)",
             device_id, status, response.data_len);
    return false;
  }
}

bool TsumikoroBridge::nucleo_get_button(uint8_t device_id, uint8_t *button_state) {
  if (!this->bus_handle_ || !button_state) {
    return false;
  }

  // Custom command 0xF003: Get button state
  tsumikoro_packet_t cmd = {
    .device_id = device_id,
    .command = 0xF003,
    .data_len = 0
  };

  tsumikoro_packet_t response;

  ESP_LOGI(TAG, ">>> Calling tsumikoro_bus_send_command_blocking for GET_BUTTON, device=0x%02X", device_id);
  tsumikoro_cmd_status_t status = tsumikoro_bus_send_command_blocking(
    this->bus_handle_, &cmd, &response, 100
  );
  ESP_LOGI(TAG, "<<< tsumikoro_bus_send_command_blocking returned status=%d", status);

  if (status == TSUMIKORO_CMD_STATUS_SUCCESS && response.data_len >= 1) {
    *button_state = response.data[0];
    ESP_LOGD(TAG, "NUCLEO_GET_BUTTON device 0x%02X: %s",
             device_id,
             *button_state ? "PRESSED" : "RELEASED");
    return true;
  } else {
    ESP_LOGW(TAG, "NUCLEO_GET_BUTTON device 0x%02X: failed (status=%d, data_len=%u)",
             device_id, status, response.data_len);
    return false;
  }
}

}  // namespace tsumikoro_bridge
}  // namespace esphome
