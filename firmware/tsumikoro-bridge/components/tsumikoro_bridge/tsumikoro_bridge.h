#pragma once

#include "esphome/core/component.h"

// Tsumikoro bus protocol headers
extern "C" {
#include "tsumikoro_bus.h"
#include "tsumikoro_hal_esp32.h"
#include "tsumikoro_protocol.h"
}

namespace esphome {
namespace tsumikoro_bridge {

/**
 * @brief Bridge component for Tsumikoro bus controller
 *
 * This component acts as a bus controller for the Tsumikoro motor control bus,
 * using the full protocol stack with FreeRTOS threading.
 */
class TsumikoroBridge : public Component {
 public:
  TsumikoroBridge() = default;

  /// Setup the component
  void setup() override;

  /// Main loop (minimal in RTOS mode)
  void loop() override;

  /// Component configuration
  void dump_config() override;

  /// Get component setup priority
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration setters (called from Python code generation)
  void set_uart_port(uint8_t port) { this->uart_port_ = port; }
  void set_tx_pin(uint8_t pin) { this->tx_pin_ = pin; }
  void set_rx_pin(uint8_t pin) { this->rx_pin_ = pin; }
  void set_de_pin(int8_t pin) { this->de_pin_ = pin; }
  void set_baud_rate(uint32_t rate) { this->baud_rate_ = rate; }
  void set_device_id(uint8_t id) { this->device_id_ = id; }
  void set_turnaround_delay(uint8_t delay) { this->turnaround_delay_ = delay; }

  /**
   * @brief Send PING command to peripheral
   * @param device_id Peripheral device ID
   * @param timeout_ms Response timeout (default: 100ms)
   * @return true if response received
   */
  bool ping_device(uint8_t device_id, uint32_t timeout_ms = 100);

  /**
   * @brief Get version from peripheral
   * @param device_id Peripheral device ID
   * @param major Output: major version
   * @param minor Output: minor version
   * @param patch Output: patch version
   * @return true if version received
   */
  bool get_version(uint8_t device_id, uint8_t *major, uint8_t *minor, uint8_t *patch);

  /**
   * @brief Get status from peripheral
   * @param device_id Peripheral device ID
   * @param status Output: status data buffer
   * @param status_len Input/Output: buffer size / actual data size
   * @return true if status received
   */
  bool get_status(uint8_t device_id, uint8_t *status, size_t *status_len);

  /**
   * @brief Send stepper move command
   * @param device_id Stepper device ID
   * @param position Target position (int32_t)
   * @return true if command sent
   */
  bool stepper_move(uint8_t device_id, int32_t position);

  /**
   * @brief Send stepper stop command
   * @param device_id Stepper device ID
   * @return true if command sent
   */
  bool stepper_stop(uint8_t device_id);

  /**
   * @brief Send servo set position command
   * @param device_id Servo device ID
   * @param position Target position (0-1800 = 0-180° * 10)
   * @return true if command sent
   */
  bool servo_set_position(uint8_t device_id, uint16_t position);

  /**
   * @brief Send servo get position command
   * @param device_id Servo device ID
   * @param position Output: current position
   * @return true if position received
   */
  bool servo_get_position(uint8_t device_id, uint16_t *position);

  /**
   * @brief Get bus statistics
   */
  tsumikoro_bus_stats_t get_bus_stats() const;

 protected:
  /// Bus unsolicited message callback (static wrapper)
  static void bus_unsolicited_callback_(const tsumikoro_packet_t *packet, void *user_data);

  /// Handle unsolicited message
  void handle_unsolicited_message_(const tsumikoro_packet_t *packet);

  // UART configuration
  uint8_t uart_port_{1};  // Default: UART1
  uint8_t tx_pin_{17};    // Default: GPIO17
  uint8_t rx_pin_{16};    // Default: GPIO16
  int8_t de_pin_{-1};     // Default: no DE pin
  uint32_t baud_rate_{1000000};  // Default: 1Mbaud

  // Bus configuration
  uint8_t device_id_{0x00};  // Controller address
  uint8_t turnaround_delay_{3};  // 3 byte intervals

  // Bus protocol handles
  tsumikoro_hal_handle_t hal_handle_{nullptr};
  tsumikoro_bus_handle_t bus_handle_{nullptr};

  // ESP32 HAL configuration
  tsumikoro_hal_esp32_config_t esp32_config_;
};

}  // namespace tsumikoro_bridge
}  // namespace esphome
