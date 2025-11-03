#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace tsumikoro_bridge {

/**
 * @brief Bridge component for communicating with Tsumikoro motor controllers
 *
 * This component handles UART communication between the ESP32 and STM32-based
 * motor controllers, providing network control and monitoring capabilities.
 */
class TsumikoroBridge : public Component, public uart::UARTDevice {
 public:
  TsumikoroBridge() = default;

  /// Setup the component
  void setup() override;

  /// Main loop
  void loop() override;

  /// Component configuration
  void dump_config() override;

  /// Get component update interval
  float get_setup_priority() const override { return setup_priority::DATA; }

  /**
   * @brief Send command to motor controller
   * @param controller_id ID of the target controller (0-255)
   * @param command Command byte
   * @param data Optional data payload
   * @param data_len Length of data payload
   * @return true if command sent successfully
   */
  bool send_command(uint8_t controller_id, uint8_t command, const uint8_t *data = nullptr, size_t data_len = 0);

  /**
   * @brief Set motor speed
   * @param controller_id ID of the target controller
   * @param speed Speed value (-32768 to 32767)
   */
  void set_motor_speed(uint8_t controller_id, int16_t speed);

  /**
   * @brief Get motor status
   * @param controller_id ID of the target controller
   * @return true if status request sent
   */
  bool get_motor_status(uint8_t controller_id);

 protected:
  /// Process incoming data from motor controllers
  void process_received_data_();

  /// Calculate checksum for packet
  uint8_t calculate_checksum_(const uint8_t *data, size_t len);

  /// Buffer for incoming data
  std::vector<uint8_t> rx_buffer_;

  /// Last communication timestamp
  uint32_t last_comm_time_{0};
};

}  // namespace tsumikoro_bridge
}  // namespace esphome
