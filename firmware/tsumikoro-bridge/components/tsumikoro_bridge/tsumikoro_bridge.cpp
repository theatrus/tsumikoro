#include "tsumikoro_bridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tsumikoro_bridge {

static const char *const TAG = "tsumikoro_bridge";

// Protocol definitions
static const uint8_t PACKET_START = 0xAA;
static const uint8_t PACKET_END = 0x55;

// Command definitions
static const uint8_t CMD_SET_SPEED = 0x01;
static const uint8_t CMD_GET_STATUS = 0x02;
static const uint8_t CMD_STOP = 0x03;
static const uint8_t CMD_RESET = 0x04;

void TsumikoroBridge::setup() {
  ESP_LOGI(TAG, "Setting up Tsumikoro Bridge...");

  // Configure UART
  this->check_uart_settings(115200);

  // Reserve buffer space
  this->rx_buffer_.reserve(256);

  ESP_LOGI(TAG, "Tsumikoro Bridge setup complete");
}

void TsumikoroBridge::loop() {
  // Process incoming data
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);

    // Process complete packets
    if (this->rx_buffer_.size() >= 4) {
      this->process_received_data_();
    }
  }

  // Check for communication timeout
  uint32_t now = millis();
  if (now - this->last_comm_time_ > 1000) {
    // Could implement timeout handling here
  }
}

void TsumikoroBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "Tsumikoro Bridge:");
  ESP_LOGCONFIG(TAG, "  UART Bus: %p", this->parent_);
}

bool TsumikoroBridge::send_command(uint8_t controller_id, uint8_t command,
                                   const uint8_t *data, size_t data_len) {
  // Packet format: [START][ID][CMD][LEN][DATA...][CHECKSUM][END]
  std::vector<uint8_t> packet;
  packet.reserve(6 + data_len);

  packet.push_back(PACKET_START);
  packet.push_back(controller_id);
  packet.push_back(command);
  packet.push_back(static_cast<uint8_t>(data_len));

  if (data && data_len > 0) {
    packet.insert(packet.end(), data, data + data_len);
  }

  uint8_t checksum = this->calculate_checksum_(packet.data() + 1, packet.size() - 1);
  packet.push_back(checksum);
  packet.push_back(PACKET_END);

  // Send packet
  this->write_array(packet.data(), packet.size());
  this->flush();

  this->last_comm_time_ = millis();

  ESP_LOGD(TAG, "Sent command 0x%02X to controller %d", command, controller_id);

  return true;
}

void TsumikoroBridge::set_motor_speed(uint8_t controller_id, int16_t speed) {
  uint8_t data[2];
  data[0] = static_cast<uint8_t>(speed >> 8);    // High byte
  data[1] = static_cast<uint8_t>(speed & 0xFF);  // Low byte

  this->send_command(controller_id, CMD_SET_SPEED, data, 2);

  ESP_LOGI(TAG, "Set motor %d speed to %d", controller_id, speed);
}

bool TsumikoroBridge::get_motor_status(uint8_t controller_id) {
  return this->send_command(controller_id, CMD_GET_STATUS, nullptr, 0);
}

void TsumikoroBridge::process_received_data_() {
  // Look for packet start
  auto start_it = std::find(this->rx_buffer_.begin(), this->rx_buffer_.end(), PACKET_START);

  if (start_it == this->rx_buffer_.end()) {
    // No start found, clear buffer
    this->rx_buffer_.clear();
    return;
  }

  // Remove data before start
  if (start_it != this->rx_buffer_.begin()) {
    this->rx_buffer_.erase(this->rx_buffer_.begin(), start_it);
  }

  // Check if we have enough data for a minimal packet
  if (this->rx_buffer_.size() < 6) {
    return;  // Wait for more data
  }

  // Extract packet fields
  uint8_t controller_id = this->rx_buffer_[1];
  uint8_t command = this->rx_buffer_[2];
  uint8_t data_len = this->rx_buffer_[3];

  // Check if we have complete packet
  size_t packet_len = 6 + data_len;  // START + ID + CMD + LEN + DATA + CHECKSUM + END

  if (this->rx_buffer_.size() < packet_len) {
    return;  // Wait for more data
  }

  // Verify checksum
  uint8_t received_checksum = this->rx_buffer_[packet_len - 2];
  uint8_t calculated_checksum = this->calculate_checksum_(
      this->rx_buffer_.data() + 1, packet_len - 3);

  if (received_checksum != calculated_checksum) {
    ESP_LOGW(TAG, "Checksum mismatch! Received: 0x%02X, Calculated: 0x%02X",
             received_checksum, calculated_checksum);
    this->rx_buffer_.erase(this->rx_buffer_.begin());
    return;
  }

  // Verify packet end
  if (this->rx_buffer_[packet_len - 1] != PACKET_END) {
    ESP_LOGW(TAG, "Invalid packet end marker");
    this->rx_buffer_.erase(this->rx_buffer_.begin());
    return;
  }

  // Process the command response
  ESP_LOGD(TAG, "Received response from controller %d, command 0x%02X",
           controller_id, command);

  // TODO: Add specific command response handling here
  // For example, publish status to Home Assistant API

  // Remove processed packet
  this->rx_buffer_.erase(this->rx_buffer_.begin(),
                         this->rx_buffer_.begin() + packet_len);
}

uint8_t TsumikoroBridge::calculate_checksum_(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

}  // namespace tsumikoro_bridge
}  // namespace esphome
