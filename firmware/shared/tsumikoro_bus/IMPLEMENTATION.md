# Tsumikoro Bus Protocol - Implementation

This directory contains the core implementation of the Tsumikoro multi-drop serial bus protocol.

## Directory Structure

```
firmware/shared/tsumikoro_bus/
├── README.md              # Design document (see main README)
├── IMPLEMENTATION.md      # This file
├── include/               # Public headers
│   ├── tsumikoro_crc8.h
│   └── tsumikoro_protocol.h
├── src/                   # Implementation
│   ├── tsumikoro_crc8.c
│   └── tsumikoro_protocol.c
└── tests/                 # Unit tests
    ├── Makefile
    ├── test_framework.h
    ├── test_crc8.c
    └── test_protocol.c
```

## Features Implemented

### Core Protocol (v0.6)
- ✅ CRC8-CCITT error detection (polynomial 0x07)
- ✅ Packet encoding/decoding
- ✅ 16-bit command support
- ✅ Variable-length data payloads (0-64 bytes)
- ✅ Generic command definitions
- ✅ Status codes and error handling

### Wire Format
```
[START(0xAA)][ID][CMD_HI][CMD_LO][LEN][DATA(0-64)][CRC8][END(0x55)]
```

- Packet size: 7-71 bytes
- Device addresses: 0x00 (controller), 0x01-0xEF (peripherals), 0xFF (broadcast)
- Commands: 16-bit organized by functional class (0x0000-0xFFFF)

## Building and Testing

### Prerequisites
- GCC compiler
- Make

### Running Tests

```bash
cd tests
make test
```

This will:
1. Compile all source files with `-Wall -Wextra -Werror`
2. Build test executables
3. Run all test suites
4. Report results

### Test Coverage

**CRC8 Tests** (14 assertions):
- Empty data handling
- Single byte CRC
- Known test vectors (including CRC8-CCITT standard "123456789" → 0xF4)
- Packet-like data
- CRC verification function
- Large data blocks (64 bytes)
- Error detection (single-bit and double-bit errors)

**Protocol Tests** (55 assertions):
- Minimum packet encoding/decoding
- Packets with data payload
- Roundtrip encode/decode
- CRC error detection
- Invalid START/END marker detection
- Buffer size validation
- Maximum data length (64 bytes)
- Helper function validation
- Device ID and command range checks

### Test Results

```
Total:  69 assertions
Passed: 69
Failed: 0
```

## Usage Example

### Encoding a Packet

```c
#include "tsumikoro_protocol.h"

// Create a PING packet to device 0x01
tsumikoro_packet_t packet = {
    .device_id = 0x01,
    .command = TSUMIKORO_CMD_PING,
    .data_len = 0
};

uint8_t buffer[TSUMIKORO_MAX_PACKET_LEN];
size_t len = tsumikoro_packet_encode(&packet, buffer, sizeof(buffer));

// Result: AA 01 00 01 00 03 55 (7 bytes)
```

### Decoding a Packet

```c
#include "tsumikoro_protocol.h"

uint8_t rx_buffer[] = { 0xAA, 0x01, 0x00, 0x01, 0x00, 0x03, 0x55 };

tsumikoro_packet_t packet;
tsumikoro_status_t status = tsumikoro_packet_decode(
    rx_buffer, sizeof(rx_buffer), &packet);

if (status == TSUMIKORO_STATUS_OK) {
    // Packet decoded successfully
    // packet.device_id = 0x01
    // packet.command = 0x0001 (CMD_PING)
    // packet.data_len = 0
}
```

### CRC8 Calculation

```c
#include "tsumikoro_crc8.h"

uint8_t data[] = { 0x01, 0x00, 0x01, 0x00 };
uint8_t crc = tsumikoro_crc8_calculate_table(data, sizeof(data));
// Result: 0x03

// Verify CRC
if (tsumikoro_crc8_verify(data, sizeof(data), 0x03)) {
    // CRC is correct
}
```

## Implementation Notes

### CRC8-CCITT
- Two implementations provided:
  - `tsumikoro_crc8_calculate()`: Bitwise (slower, smaller code)
  - `tsumikoro_crc8_calculate_table()`: Table-based (faster, 256 bytes)
- Use bitwise for STM32G030 (32KB flash)
- Use table-based for STM32G071 and ESP32

### Platform Independence
- Pure C11 implementation
- No platform-specific dependencies in core protocol
- Uses standard library: `<stdint.h>`, `<stddef.h>`, `<stdbool.h>`, `<string.h>`
- Ready for integration with STM32 HAL, ESP32 IDF, or Linux/POSIX

### Error Handling
- All functions validate inputs (NULL pointers, buffer sizes)
- Return 0 or error codes on failure
- CRC errors detected and reported
- Invalid packet markers caught

## Next Steps

To complete the full bus implementation:

1. **HAL Layer** - Platform-specific UART/DMA interfaces
   - `hal/stm32/` - STM32 HAL UART + DMA
   - `hal/esp32/` - ESP32 IDF UART driver
   - `hal/host/` - POSIX termios
   - `hal/mock/` - Mock HAL for testing

2. **Bus Management** - Higher-level bus functions
   - TX/RX state machines
   - Collision detection (CSMA/CD)
   - Timeout handling
   - Retry logic with exponential backoff

3. **Command Dispatch** - Command handler registration
   - Command handler table
   - Response building
   - Broadcast handling

4. **Device Discovery** - Bus scanning
   - Ping sweep implementation
   - Device information queries
   - Health monitoring

## License

Copyright (c) 2025 Yann Ramin

SPDX-License-Identifier: Apache-2.0
