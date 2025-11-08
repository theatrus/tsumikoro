# Tsumikoro Multi-Drop Serial Bus Library

## Design Document v0.7

**Status**: Draft - Design Phase
**Last Updated**: 2025-11-08
**Author**: Tsumikoro Project
**Copyright**: 2025-2025 Yann Ramin
**License**: Apache-2.0

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Requirements](#2-requirements)
3. [Architecture](#3-architecture)
4. [Bus Topology & Protocol](#4-bus-topology--protocol)
5. [DMA Integration](#5-dma-integration)
6. [Hardware Abstraction Layer](#6-hardware-abstraction-layer)
7. [API Design](#7-api-design)
8. [State Machine](#8-state-machine)
9. [Error Handling & Recovery](#9-error-handling--recovery)
10. [Memory Management](#10-memory-management)
11. [Thread Safety & Concurrency](#11-thread-safety--concurrency)
12. [Testing Strategy](#12-testing-strategy)
13. [Performance Considerations](#13-performance-considerations)
14. [Configuration Options](#14-configuration-options)
15. [Future Enhancements](#15-future-enhancements)
16. [References](#16-references)

---

## 1. Introduction

### 1.1 Purpose

The Tsumikoro Multi-Drop Serial Bus Library provides efficient, DMA-based communication between motor controllers and bridge devices over a shared serial bus. It enables multiple STM32-based motor controllers (tsumikoro-ministepper, tsumikoro-servo) to communicate with an ESP32 bridge over a single half-duplex UART connection.

### 1.2 Scope

This library supports:
- **STM32G0xx** microcontrollers (STM32G030, STM32G071)
- **ESP32/ESP32-S3** wireless bridge controllers
- **Host PC/Linux** for unit testing and development
- **Mock HAL** for automated testing without hardware

### 1.3 Goals

- **Low CPU overhead**: <5% utilization using DMA-based I/O
- **Minimal RAM usage**: <512 bytes on resource-constrained MCUs (STM32G030)
- **Deterministic timing**: <2ms latency for command-response cycles
- **High reliability**: >99.9% packet success rate with CRC8 error detection
- **Platform portability**: Clean abstraction layer for multiple platforms
- **Testability**: Full unit testing on host systems without hardware

### 1.4 Non-Goals

- High-speed protocols (>2Mbaud)
- Mesh networking or ad-hoc routing
- Wireless protocols (handled by ESP32 bridge separately)
- Real-time guarantees (soft real-time only)

---

## 2. Requirements

### 2.1 Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-001 | Support multi-drop bus with up to 254 devices | MUST |
| FR-002 | Half-duplex UART operation at 1Mbaud | MUST |
| FR-003 | DMA-based transmission and reception | MUST |
| FR-004 | CRC8 error detection for all packets | MUST |
| FR-005 | Platform abstraction (STM32, ESP32, Host) | MUST |
| FR-006 | Controller-peripheral and peer-to-peer topologies | SHOULD |
| FR-007 | Automatic retry with exponential backoff | SHOULD |
| FR-008 | Broadcast addressing (all devices) | SHOULD |
| FR-009 | Unit testable on host PC/Linux | MUST |
| FR-010 | Integration with existing tsumikoro_protocol.h | MUST |

### 2.2 Non-Functional Requirements

| ID | Requirement | Target | Platform |
|----|-------------|--------|----------|
| NFR-001 | CPU overhead | <5% | All |
| NFR-002 | RAM usage | <512 bytes | STM32G030 |
| NFR-002 | RAM usage | <2KB | STM32G071 |
| NFR-002 | RAM usage | <8KB | ESP32-S3 |
| NFR-003 | Latency (command-response) | <2ms | All |
| NFR-004 | Packet error rate | <0.1% | All |
| NFR-005 | Bus utilization | >80% | All |
| NFR-006 | Code size | <8KB | STM32 |
| NFR-007 | Deterministic behavior | Yes | Controller mode |

### 2.3 Platform Constraints

| Platform | Flash | RAM | Clock | Notes |
|----------|-------|-----|-------|-------|
| STM32G030F6 | 32KB | 8KB | 64MHz | Smallest target, bitwise CRC8 |
| STM32G071G8 | 64KB | 36KB | 64MHz | Table-based CRC8 |
| ESP32-S3 | 8MB | 512KB | 240MHz | Bridge/controller role |
| Host/Linux | N/A | N/A | N/A | Testing only |

---

## 3. Architecture

### 3.1 Layered Design

The library follows a classic layered network stack:

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   (User code, callbacks)            │
├─────────────────────────────────────┤
│   Transport Layer                   │
│   (Message queue, reliability)      │
├─────────────────────────────────────┤
│   Data Link Layer                   │
│   (Packet framing, CRC8)            │
├─────────────────────────────────────┤
│   Physical Layer                    │
│   (UART/DMA operations)             │
├─────────────────────────────────────┤
│   Hardware Abstraction Layer        │
│   (STM32 HAL / ESP32 IDF / POSIX)  │
└─────────────────────────────────────┘
```

### 3.2 Component Diagram

```
[User Application]
       ↓↑
[tsumikoro_bus API]
       ↓↑
[Message Queue] ← [State Machine]
       ↓↑              ↓↑
[Packet Parser] [Packet Builder]
       ↓↑              ↓↑
[DMA RX Buffer] [DMA TX Buffer]
       ↓↑              ↓↑
[Platform HAL - STM32 | ESP32 | Host | Mock]
       ↓↑
[UART Hardware / Virtual]
```

### 3.3 File Structure

```
firmware/shared/tsumikoro_bus/
├── README.md                    # This document
├── tsumikoro_bus.h              # Main API header
├── tsumikoro_bus.c              # Platform-independent logic
├── tsumikoro_bus_config.h       # Configuration options
├── tsumikoro_bus_internal.h     # Internal state/structures
├── hal/
│   ├── tsumikoro_bus_hal.h      # HAL interface definition
│   ├── stm32/
│   │   ├── tsumikoro_bus_hal_stm32.c
│   │   └── tsumikoro_bus_hal_stm32.h
│   ├── esp32/
│   │   ├── tsumikoro_bus_hal_esp32.c
│   │   └── tsumikoro_bus_hal_esp32.h
│   ├── host/
│   │   ├── tsumikoro_bus_hal_host.c    # POSIX/termios
│   │   └── tsumikoro_bus_hal_host.h
│   └── mock/
│       ├── tsumikoro_bus_hal_mock.c    # Unit testing
│       └── tsumikoro_bus_hal_mock.h
├── tests/
│   ├── CMakeLists.txt           # Test build configuration
│   ├── test_protocol.c          # Protocol layer tests
│   ├── test_state_machine.c     # State machine tests
│   ├── test_dma_buffers.c       # Buffer management tests
│   ├── test_crc8.c              # CRC8 implementation tests
│   └── test_integration.c       # End-to-end tests
└── CMakeLists.txt               # Build configuration
```

### 3.4 Platform Support

| Platform | Use Case | UART Interface | DMA Support | Test Support |
|----------|----------|----------------|-------------|--------------|
| STM32G0xx | Motor controller | HAL UART | HAL DMA | Via SWD |
| ESP32/S3 | Bridge/Controller | IDF UART | ESP-DMA | Via JTAG |
| Linux/macOS | Unit testing | termios/POSIX | Simulated | Native |
| Mock HAL | Pure unit tests | In-memory | Simulated | Native |

---

## 4. Bus Topology & Protocol

### 4.1 Topology Options

#### Option A: Controller-Peripheral (RECOMMENDED)

```
                    RS-485 Bus (Half-Duplex)
    ┌──────┬────────────────────────────────────┬────────┐
    │      │                                    │        │
┌───┴───┐ ┌┴────────┐ ┌──────────┐ ┌──────────┐ ┌─────┴──┐
│ESP32  │ │ STM32   │ │  STM32   │ │  STM32   │ │ STM32  │
│Bridge │ │ Motor 1 │ │ Motor 2  │ │ Motor 3  │ │ ...254 │
│(Ctrlr)│ │ (Periph)│ │ (Periph) │ │ (Periph) │ │(Periph)│
│ID=0x00│ │ ID=0x01 │ │ ID=0x02  │ │ ID=0x03  │ │        │
└───────┘ └─────────┘ └──────────┘ └──────────┘ └────────┘
```

**Characteristics**:
- ESP32 bridge acts as controller (ID=0x00)
- STM32 motor controllers are peripherals (ID=0x01-0xFE)
- Controller initiates all transactions
- Peripherals only transmit when queried
- Deterministic timing (polling-based)
- Simple collision avoidance

**Pros**:
- Deterministic latency
- Simple implementation
- No collision handling needed
- Ideal for motor control

**Cons**:
- Single point of failure (controller)
- Controller must poll all peripherals
- Scalability limited by polling overhead

#### Option B: Peer-to-Peer with CSMA/CD

```
                    RS-485 Bus (Half-Duplex)
    ┌──────┬────────────────┬────────────────┬────────┐
    │      │                │                │        │
┌───┴───┐ ┌┴────────┐ ┌────┴──────┐ ┌──────┴──┐ ┌───┴────┐
│Device │ │ Device  │ │  Device   │ │ Device  │ │ Device │
│  #1   │ │   #2    │ │    #3     │ │   #4    │ │  ...N  │
│(Peer) │ │ (Peer)  │ │  (Peer)   │ │ (Peer)  │ │ (Peer) │
└───────┘ └─────────┘ └───────────┘ └─────────┘ └────────┘
```

**Characteristics**:
- All devices are peers
- Carrier Sense Multiple Access with Collision Detection
- Listen-before-transmit
- Random backoff on collision
- Priority levels based on device ID

**Pros**:
- No single point of failure
- More flexible topology
- Better for sensor networks

**Cons**:
- Non-deterministic latency
- More complex collision handling
- Lower effective throughput

### 4.2 Addressing Scheme

| Address Range | Purpose |
|---------------|---------|
| 0x00 | Controller/Bridge (ESP32) |
| 0x01 - 0xEF | Standard device addresses (239 devices) |
| 0xF0 - 0xFE | Reserved for future use |
| 0xFF | Broadcast address (all devices) |

### 4.3 Bus Arbitration (Controller-Peripheral)

**Polling Strategy**:
1. Controller maintains a list of active peripheral IDs
2. Controller polls each peripheral in round-robin order
3. Peripheral responds within timeout window (20ms)
4. If no response, mark peripheral as offline (after N retries)
5. Continue to next peripheral

**Timing**:
- Command packet TX: ~700µs (70 bytes at 1Mbaud)
- Response packet TX: ~700µs
- Processing delay: ~500µs
- **Total transaction**: ~2ms per device
- **10 devices**: ~20ms polling cycle

### 4.4 Half-Duplex Control

#### Hardware Approach (RECOMMENDED)

Use RS-485 transceiver with Driver Enable (DE) and Receiver Enable (RE) pins:

```
STM32/ESP32              RS-485 Transceiver
┌─────────┐              ┌──────────┐
│ UART_TX ├─────────────>│ DI   RO  ├───> To other device RX
│ UART_RX │<─────────────┤ RO   DI  │<─── From other device TX
│ GPIO_DE ├─────────────>│ DE       │
│ GPIO_RE ├─────────────>│ /RE      │
└─────────┘              └──────────┘
```

**Control Logic**:
- Before TX: Set DE=1, RE=0 (driver enabled, receiver disabled)
- During TX: Transmit data
- After TX: Set DE=0, RE=1 (driver disabled, receiver enabled)
- During RX: Keep DE=0, RE=1

**Timing Requirements**:
- DE setup time: ~1µs before first byte
- DE hold time: ~1µs after last byte
- Handled automatically in HAL layer

#### Software Approach

- Disable RX during TX (software control)
- Timing-based switching
- Less reliable, not recommended for production

#### 4.4.1 TX Echo and Collision Detection

**CRITICAL**: The half-duplex bus operates with **TX echo** enabled. All transmitters receive their own transmitted data on the bus.

**RS-485 Bus Behavior**:
- When a device transmits, the data appears on the shared bus (A/B differential lines)
- **All devices** (including the transmitter) receive this data via their RX line
- The transmitter receives an "echo" of its own transmission
- This echo is used for **collision detection**

```
Device A transmits:              Device B (listening):
  TX: AA 01 22 ...                RX: AA 01 22 ...
  RX: AA 01 22 ... (echo)         (receives from bus)
       └─ Compare with TX
```

**Collision Detection Mechanism**:

When transmitting, a device must:
1. **Enable TX mode** (set DE=1, RE can stay enabled for echo)
2. **Transmit data** byte-by-byte via UART TX
3. **Simultaneously read RX** to receive the echo
4. **Compare transmitted vs received** byte-by-byte
5. **Detect collision** if bytes don't match

```c
/**
 * @brief Transmit packet with collision detection
 * @return true if transmission successful, false if collision detected
 */
bool tsumikoro_bus_transmit_with_collision_detect(
    const uint8_t *tx_data,
    size_t tx_len
) {
    uint8_t rx_echo[70];  // Buffer for echo

    // Enable TX mode (DE=1, keep RE=1 for echo)
    tsumikoro_hal_enable_tx_with_echo();

    // Transmit and receive echo simultaneously
    for (size_t i = 0; i < tx_len; i++) {
        // Send byte
        uart_send_byte(tx_data[i]);

        // Wait for echo
        uint32_t timeout = get_time_us() + 100;  // 100µs timeout per byte
        while (!uart_rx_available()) {
            if (get_time_us() > timeout) {
                // Echo timeout - possible bus fault
                tsumikoro_hal_enable_rx();
                return false;
            }
        }

        // Read echo
        rx_echo[i] = uart_recv_byte();

        // Compare transmitted vs echo
        if (rx_echo[i] != tx_data[i]) {
            // COLLISION DETECTED!
            // Another device is transmitting simultaneously
            tsumikoro_hal_enable_rx();
            tsumikoro_bus_handle_collision();
            return false;
        }
    }

    // All bytes matched - successful transmission
    tsumikoro_hal_enable_rx();
    return true;
}
```

**RS-485 Transceiver Configuration for Echo**:

Most RS-485 transceivers support simultaneous TX and RX:

```
Transceiver Mode Table:
┌────┬─────┬──────────────────────────────────┐
│ DE │ /RE │ Mode                             │
├────┼─────┼──────────────────────────────────┤
│ 0  │  0  │ RX enabled (normal receive)      │
│ 1  │  1  │ TX enabled, RX disabled (no echo)│
│ 1  │  0  │ TX + RX enabled (with echo) ✓    │
│ 0  │  1  │ High-Z (disabled)                │
└────┴─────┴──────────────────────────────────┘

Recommended: DE=1, /RE=0 during transmission (enables echo)
```

**Hardware-Assisted Echo** (some transceivers):
- Some RS-485 transceivers have automatic echo on TX
- Others require explicit RE=0 during transmission
- Check datasheet for specific transceiver behavior

#### 4.4.2 Bus Collision Handling

When a collision is detected (TX echo mismatch), **all nodes must back off**:

**Collision Resolution Protocol** (CSMA/CD - Carrier Sense Multiple Access with Collision Detection):

1. **Detect Collision**: TX byte ≠ RX echo byte
2. **Abort Transmission**: Stop sending immediately
3. **Signal Collision** (optional): Send jamming signal (0xAA pattern) to ensure all nodes detect collision
4. **Random Backoff**: Wait random time before retry
5. **Retry Transmission**: Attempt again after backoff

```c
/**
 * @brief Handle bus collision
 */
void tsumikoro_bus_handle_collision(void) {
    // Increment collision counter
    bus_stats.collisions++;

    // Optional: Send jamming signal to ensure all nodes detect collision
    // (Some protocols send short burst of 0xAA)

    // Random backoff: Use device ID and random number for backoff time
    // This ensures devices don't retry simultaneously
    uint32_t random_factor = rand() % 16;  // 0-15
    uint32_t device_priority = 255 - bus_config.device_id;  // Lower ID = higher priority

    // Backoff time: base + random + priority
    // Base: 500µs (minimum quiet time)
    // Random: 0-150µs (prevents synchronous retries)
    // Priority: 0-255µs (lower device IDs retry sooner)
    uint32_t backoff_us = 500 + (random_factor * 10) + device_priority;

    // Wait for backoff period
    delay_us(backoff_us);

    // Listen before transmitting (carrier sense)
    while (tsumikoro_hal_bus_is_active()) {
        delay_us(100);  // Wait for bus to become idle
    }

    // Additional random delay to prevent re-collision
    delay_us(rand() % 100);
}
```

**Backoff Strategy**:

```
Collision detected at byte N:
  │
  ├─ Stop transmission immediately
  ├─ Calculate backoff time:
  │    Base: 500µs (minimum quiet time)
  │    Random: 0-150µs (rand() % 16 * 10)
  │    Priority: (255 - device_id) µs
  │    Total: 500-905µs
  │
  ├─ Wait backoff period
  ├─ Sense carrier (check if bus is idle)
  ├─ Wait additional random 0-100µs
  └─ Retry transmission

Priority Example (collision at same time):
  Device 0x01: backoff = 500 + 80 + 254 = 834µs
  Device 0x10: backoff = 500 + 120 + 245 = 865µs
  Device 0xFF: backoff = 500 + 50 + 0 = 550µs

  → Device 0xFF retries first (broadcast has lowest priority)
  → Device 0x01 retries last (controller has highest priority)
```

**Exponential Backoff** (for repeated collisions):

```c
int collision_count = 0;
int base_backoff_us = 500;

while (collision_count < MAX_COLLISION_RETRIES) {
    if (tsumikoro_bus_transmit_with_collision_detect(packet, len)) {
        // Success
        collision_count = 0;
        return TSUMIKORO_STATUS_OK;
    }

    // Collision detected
    collision_count++;

    // Exponential backoff: multiply base by 2^collision_count
    uint32_t backoff = base_backoff_us * (1 << collision_count);
    backoff += rand() % (backoff / 2);  // Add random jitter

    delay_us(backoff);

    // Carrier sense before retry
    while (tsumikoro_hal_bus_is_active()) {
        delay_us(100);
    }
}

// Too many collisions - abort
return TSUMIKORO_STATUS_BUS_BUSY;
```

**Backoff Times** (exponential):
- 1st collision: 500µs + jitter
- 2nd collision: 1ms + jitter
- 3rd collision: 2ms + jitter
- 4th collision: 4ms + jitter
- 5th collision: 8ms + jitter (abort)

#### 4.4.3 Controller-Peripheral vs Peer-to-Peer

**Controller-Peripheral Mode**:
- Collision detection still recommended for safety
- Collisions indicate protocol violation (peripheral talking out of turn)
- Controller should log collision as error condition
- Peripheral should never initiate transmission (only respond)

**Peer-to-Peer Mode**:
- Collision detection is **mandatory**
- All nodes are equal, can initiate transmission
- CSMA/CD protocol required for bus arbitration
- Random backoff prevents livelock

**Configuration**:

```c
// Controller-Peripheral mode (collision detection for safety only)
tsumikoro_bus_config_t config = {
    .is_controller = true,  // or false for peripheral
    .enable_collision_detect = true,  // Recommended
    .collision_retry_count = 1,  // Don't retry much (protocol error)
};

// Peer-to-Peer mode (collision detection required)
tsumikoro_bus_config_t config = {
    .is_controller = false,  // All nodes are peers
    .enable_collision_detect = true,  // REQUIRED
    .collision_retry_count = 5,  // Retry with exponential backoff
};
```

#### 4.4.4 Carrier Sense (Listen Before Transmit)

Before any transmission, nodes should check if the bus is idle:

```c
/**
 * @brief Check if bus has carrier (active transmission)
 * @return true if bus is active (another device transmitting)
 */
bool tsumikoro_hal_bus_is_active(void) {
    // Method 1: Check UART RX line activity
    // If RX is receiving data, bus is active
    if (uart_rx_available()) {
        return true;
    }

    // Method 2: Monitor differential voltage on RS-485 A/B lines
    // (Requires additional hardware comparator)

    // Method 3: Check for START marker in recent RX data
    // If START (0xAA) seen recently, bus may be active

    return false;
}

/**
 * @brief Wait for bus to become idle
 * @param timeout_ms Maximum wait time
 * @return true if bus became idle, false if timeout
 */
bool tsumikoro_bus_wait_idle(uint32_t timeout_ms) {
    uint32_t start = get_time_ms();

    while (tsumikoro_hal_bus_is_active()) {
        if ((get_time_ms() - start) > timeout_ms) {
            return false;  // Timeout
        }
        delay_us(100);
    }

    // Bus is idle, wait additional guard time
    delay_us(200);  // 200µs guard time

    return true;
}
```

**Transmission Flow with Carrier Sense**:

```
1. Application calls send_packet()
   │
2. Check if bus is idle (carrier sense)
   │
   ├─ Bus active? → Wait for idle (up to timeout)
   │
3. Bus idle → Assert DE=1, enable TX
   │
4. Transmit bytes with echo comparison
   │
   ├─ Echo mismatch? → COLLISION!
   │    └─ Abort, backoff, retry
   │
5. All bytes transmitted successfully
   │
6. De-assert DE=0, return to RX mode
   │
7. Return success
```

### 4.5 Error Detection: CRC8-CCITT

**Polynomial**: CRC-8-CCITT (0x07)
- Polynomial: x^8 + x^2 + x + 1
- Initial value: 0x00
- Final XOR: 0x00
- Reflects: No

**Why CRC8?**

At 1Mbaud, motor noise and EMI can cause burst errors. CRC8 provides superior error detection compared to simple XOR checksum:

| Error Type | XOR Detects | CRC8 Detects |
|------------|-------------|--------------|
| Single bit flip | ✓ 100% | ✓ 100% |
| Two bits same position | ✗ ~50% | ✓ 100% |
| Burst errors (≤8 bits) | ✗ ~50% | ✓ 100% |
| Random errors | ✗ ~50% | ✓ 99.6% |

**Error Detection Guarantees** (for packets ≤70 bytes):
- ✓ 100% detection of all single-bit errors
- ✓ 100% detection of all double-bit errors
- ✓ 100% detection of all odd number of bit errors
- ✓ 100% detection of all burst errors ≤8 bits
- ✓ 99.6% detection of all other error patterns

### 4.6 Packet Format

Based on existing `tsumikoro_protocol.h` with CRC8 checksum, **16-bit command field**, and **byte stuffing** for frame synchronization:

```
Wire Format (after byte stuffing):
[START(2)][ID][CMD_HI][CMD_LO][LEN][DATA(0-64)][CRC8][END(1)]

Byte Offsets (unstuffed):
  0-1: START marker (0xAA 0xAA) - double byte for unambiguous frame detection
  2: Controller ID (0x00-0xFF)
  3: Command high byte (tsumikoro_command_t >> 8)
  4: Command low byte (tsumikoro_command_t & 0xFF)
  5: Data length (0-64)
  6 to 6+len-1: Data payload
  6+len: CRC8 checksum (covers bytes 2 through 5+len)
  7+len: END marker (0x55)

Minimum packet size: 8 bytes (no data)
Maximum packet size: 149 bytes (with full byte stuffing of 64-byte payload)
Typical packet size: 8 + data_len bytes (unstuffed)
```

**Byte Stuffing (v0.7)**:

To prevent false frame synchronization, bytes 0xAA and 0x55 in the payload are escaped:

```
Escape Sequences:
  0xAA in payload → 0xAA 0x01
  0x55 in payload → 0xAA 0x02

Escape byte: 0xAA (same as START marker)
```

**Important**: Byte stuffing applies ONLY to the payload (ID, CMD_HI, CMD_LO, LEN, DATA, CRC8).
The START marker (0xAA 0xAA) and END marker (0x55) are NOT escaped.

**Byte Order**: Little-endian (CMD_HI first, CMD_LO second)

**CRC8 Calculation Range** (on unstuffed data):
- Starts at: `controller_id` (byte 2)
- Ends at: Last data byte (byte 5+len)
- Includes: ID + CMD_HI + CMD_LO + LEN + DATA
- CRC is calculated BEFORE byte stuffing

**Example Packet** (SET_SPEED command 0x2002 to device 0x01, speed=100):
```
Hex:  AA AA 01 20 02 02 00 64 [CRC8] 55
      │  │  │  │  │  │  └─┬─┘   │     │
      │  │  │  │  │  │    │     │     └─ END marker
      │  │  │  │  │  │    │     └─────── CRC8 of [01 20 02 02 00 64]
      │  │  │  │  │  │    └───────────── Data: 0x0064 = 100 (little-endian)
      │  │  │  │  │  └────────────────── Length: 2 bytes
      │  │  │  │  └───────────────────── Command low: 0x02
      │  │  │  └──────────────────────── Command high: 0x20 (stepper range)
      │  │  └─────────────────────────── Controller ID: 1
      │  └────────────────────────────── START marker (byte 2)
      └───────────────────────────────── START marker (byte 1)

Command: 0x2002 (Stepper: SET_SPEED)
Total: 10 bytes (no byte stuffing needed in this example)
```

**Broadcast Example** (STOP command 0x0003 to all devices):
```
Hex:  AA AA FF 00 03 00 [CRC8] 55
      │  │  │  │  │  │    │     │
      │  │  │  │  │  │    │     └─ END marker
      │  │  │  │  │  │    └─────── CRC8 of [FF 00 03 00]
      │  │  │  │  │  └──────────── Length: 0 bytes (no data)
      │  │  │  │  └─────────────── Command low: 0x03 (STOP)
      │  │  │  └────────────────── Command high: 0x00 (generic range)
      │  │  └───────────────────── Controller ID: 0xFF (broadcast)
      │  └──────────────────────── START marker (byte 2)
      └─────────────────────────── START marker (byte 1)

Command: 0x0003 (Generic: STOP)
Total: 8 bytes (minimum packet size)
```

**Byte Stuffing Example** (packet with 0xAA and 0x55 in payload):
```
Unstuffed (logical):
  AA AA 10 F0 03 01 AA 34 55    (device 0x10, cmd 0xF003, data=[0xAA])
        │              └─ This 0xAA in payload needs escaping

Stuffed (on wire):
  AA AA 10 F0 03 01 AA 01 34 55
        │              └─┬─┘      Escaped: 0xAA → 0xAA 0x01

Total: 10 bytes on wire (9 bytes unstuffed + 1 escape byte)
```

### 4.7 Command/Response Protocol

The bus operates on a **command/response model** with timeout-based error detection:

```
Controller                   Peripheral
  │                               │
  ├──── Command Packet ──────────>│
  │                               │ Process command
  │                               │ Build response
  │<──── Response Packet ─────────┤
  │                               │
  └─ Timeout (20ms) if no response

Broadcast Command (no response expected):
  │                               │
  ├──── Broadcast Packet ────────>│ All devices
  │                               │ Execute command
  │     (no response)             │ No response
  │                               │
```

**Key Principles**:
- Every command expects a response (except broadcasts to 0xFF)
- Response timeout: 20ms (configurable)
- Failed commands retry with exponential backoff
- Responses reuse the command byte to correlate request/response

#### 4.7.1 Command Types

Commands use **16-bit identifiers** (0x0000-0xFFFF) divided by functional class:

**Command Range Allocation**:

| Range | Class | Description |
|-------|-------|-------------|
| 0x0000-0x0FFF | Generic | Common to all device types |
| 0x1000-0x1FFF | Sensor | Sensor devices (temp, proximity, etc.) |
| 0x2000-0x2FFF | Stepper | Stepper motor controllers |
| 0x3000-0x3FFF | Servo | Servo motor controllers |
| 0x4000-0x4FFF | DC Motor | DC motor controllers |
| 0x5000-0x5FFF | Bridge | Bridge/gateway devices |
| 0x6000-0xEFFF | Reserved | Future device classes |
| 0xF000-0xFFFF | Custom | User-defined commands |

**Generic Commands** (0x0000-0x0FFF): Common across all device types

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_PING | 0x0001 | Ping device | ACK with device info |
| CMD_GET_INFO | 0x0002 | Get device information | Device info structure |
| CMD_STOP | 0x0003 | Emergency stop | ACK |
| CMD_RESET | 0x0004 | Reset device | ACK |
| CMD_BOOTLOADER | 0x0005 | Enter bootloader mode | ACK |
| CMD_GET_VERSION | 0x0006 | Get firmware version | Version structure |
| CMD_SET_DEVICE_ID | 0x0007 | Change device ID | ACK |
| CMD_SAVE_CONFIG | 0x0008 | Save configuration to flash | ACK |
| CMD_LOAD_CONFIG | 0x0009 | Load configuration from flash | ACK |
| CMD_GET_STATUS | 0x000A | Get device status | Status structure |
| CMD_ID_ASSIGN_START | 0x000B | Start hardware ID assignment (OPTIONAL) | Broadcast (no response) |
| CMD_ID_ASSIGN_ACK | 0x000C | Peripheral acknowledges assigned ID (OPTIONAL) | Controller receives ACK |
| CMD_ID_ASSIGN_COMPLETE | 0x000D | ID assignment complete (OPTIONAL) | Broadcast (no response) |

**Sensor Commands** (0x1000-0x1FFF): Sensor devices

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_GET_VALUE | 0x1001 | Get sensor value | Sensor data |
| CMD_GET_CALIBRATION | 0x1002 | Get calibration data | Calibration structure |
| CMD_SET_CALIBRATION | 0x1003 | Set calibration data | ACK |
| CMD_ZERO | 0x1004 | Zero/tare sensor | ACK |
| CMD_SET_THRESHOLD | 0x1005 | Set threshold value | ACK |
| CMD_GET_THRESHOLD | 0x1006 | Get threshold value | Threshold data |

**Stepper Motor Commands** (0x2000-0x2FFF): Stepper motor controllers

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_SET_POSITION | 0x2001 | Set target position | ACK |
| CMD_GET_POSITION | 0x2002 | Get current position | Position (int32_t) |
| CMD_SET_SPEED | 0x2003 | Set movement speed | ACK |
| CMD_GET_SPEED | 0x2004 | Get current speed | Speed (int16_t) |
| CMD_HOME | 0x2005 | Home to limit switch | ACK |
| CMD_ENABLE | 0x2006 | Enable motor | ACK |
| CMD_DISABLE | 0x2007 | Disable motor | ACK |
| CMD_SET_CURRENT | 0x2008 | Set motor current | ACK |
| CMD_GET_CURRENT | 0x2009 | Get motor current | Current (uint16_t) |
| CMD_SET_MICROSTEPPING | 0x200A | Set microstepping mode | ACK |
| CMD_SET_ACCELERATION | 0x200B | Set acceleration | ACK |

**Servo Motor Commands** (0x3000-0x3FFF): Servo motor controllers

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_SET_ANGLE | 0x3001 | Set target angle | ACK |
| CMD_GET_ANGLE | 0x3002 | Get current angle | Angle (int16_t) |
| CMD_SET_SPEED | 0x3003 | Set movement speed | ACK |
| CMD_GET_SPEED | 0x3004 | Get movement speed | Speed (uint16_t) |
| CMD_SET_LIMITS | 0x3005 | Set angle limits | ACK |
| CMD_GET_LIMITS | 0x3006 | Get angle limits | Limits structure |
| CMD_CALIBRATE | 0x3007 | Calibrate servo | ACK |
| CMD_ENABLE | 0x3008 | Enable servo | ACK |
| CMD_DISABLE | 0x3009 | Disable servo | ACK |

**DC Motor Commands** (0x4000-0x4FFF): DC motor controllers

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_SET_SPEED | 0x4001 | Set motor speed (-100 to +100%) | ACK |
| CMD_GET_SPEED | 0x4002 | Get current speed | Speed (int8_t) |
| CMD_SET_PWM | 0x4003 | Set PWM duty cycle | ACK |
| CMD_ENABLE | 0x4004 | Enable motor | ACK |
| CMD_DISABLE | 0x4005 | Disable motor | ACK |
| CMD_BRAKE | 0x4006 | Brake motor | ACK |
| CMD_COAST | 0x4007 | Coast motor (free-wheel) | ACK |

**Bridge/Gateway Commands** (0x5000-0x5FFF): Bridge devices (ESP32)

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_SCAN | 0x5001 | Scan for devices on bus | Device list |
| CMD_GET_STATS | 0x5002 | Get bus statistics | Stats structure |
| CMD_RESET_STATS | 0x5003 | Reset statistics | ACK |
| CMD_SET_BAUD_RATE | 0x5004 | Set bus baud rate | ACK |
| CMD_GET_DEVICE_LIST | 0x5005 | Get list of online devices | Device IDs |

#### 4.7.2 Response Types

All responses use the same packet format as commands, with the **command byte** indicating the original command.

**Response Patterns**:

1. **ACK Response** (write commands, no data to return):
   ```
   Request:  AA 01 22 02 00 64 [CRC8] 55  (SET_SPEED, speed=100)
   Response: AA 01 22 01 00 [CRC8] 55     (ACK, status=OK)
                      └─ Status byte: 0x00 = OK
   ```

2. **NAK Response** (command failed):
   ```
   Request:  AA 01 20 04 00 00 10 00 [CRC8] 55  (SET_POSITION, pos=4096)
   Response: AA 01 20 01 01 [CRC8] 55            (NAK, error=INVALID_DATA)
                      └─ Status byte: 0x01 = ERROR
   ```

3. **Data Response** (read commands):
   ```
   Request:  AA 01 21 00 [CRC8] 55        (GET_POSITION, no data)
   Response: AA 01 21 05 00 00 00 10 00 [CRC8] 55  (Position, status + int32_t)
                      └─ Status byte (0x00) + position (0x00001000 = 4096)
   ```

**Response Data Format**:
- First byte of data is always **status code** (0x00 = OK, >0 = error)
- Remaining bytes are command-specific payload
- Variable length (0-63 bytes after status)

**Status Codes** (first byte of response data):

| Code | Name | Description |
|------|------|-------------|
| 0x00 | STATUS_OK | Command successful |
| 0x01 | STATUS_ERROR | Generic error |
| 0x02 | STATUS_INVALID_CMD | Unknown command |
| 0x03 | STATUS_INVALID_DATA | Invalid data or parameters |
| 0x04 | STATUS_BUSY | Device busy, retry later |
| 0x05 | STATUS_FAULT | Hardware fault condition |
| 0x06 | STATUS_TIMEOUT | Internal timeout |
| 0x07 | STATUS_NOT_READY | Device not initialized |

#### 4.7.3 Command Dispatch Mechanism

Each device implements a **command handler table** for dispatching commands:

```c
/**
 * @brief Command handler function type
 * @param cmd Command word (16-bit)
 * @param request_data Request data payload
 * @param request_len Request data length
 * @param response_data Buffer for response data
 * @param response_len Pointer to response data length (output)
 * @return Status code (0x00 = OK, >0 = error)
 */
typedef uint8_t (*tsumikoro_cmd_handler_t)(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
);

/**
 * @brief Command handler registration entry
 */
typedef struct {
    uint16_t cmd;                     // Command word to handle (16-bit)
    tsumikoro_cmd_handler_t handler;  // Handler function
    const char *name;                 // Command name (debug)
} tsumikoro_cmd_entry_t;

/**
 * @brief Register command handler
 * @param bus Bus handle
 * @param cmd Command word (16-bit)
 * @param handler Handler function
 * @param name Command name (for debugging)
 */
void tsumikoro_bus_register_command(
    tsumikoro_bus_handle_t bus,
    uint16_t cmd,
    tsumikoro_cmd_handler_t handler,
    const char *name
);
```

**Example Handler Implementation**:

```c
// Stepper motor command definitions (0x2000 range)
#define CMD_GET_POSITION  0x2002
#define CMD_SET_SPEED     0x2003

// Handler for GET_POSITION command (0x2002)
uint8_t handle_get_position(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // No request data expected for GET_POSITION
    if (request_len != 0) {
        *response_len = 0;
        return STATUS_INVALID_DATA;
    }

    // Read current position from hardware
    int32_t position = stepper_get_position();

    // Build response: status (1 byte) + position (4 bytes)
    response_data[0] = STATUS_OK;
    memcpy(&response_data[1], &position, sizeof(int32_t));
    *response_len = 5;

    return STATUS_OK;
}

// Handler for SET_SPEED command (0x2003)
uint8_t handle_set_speed(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // Expect 2 bytes (int16_t speed)
    if (request_len != 2) {
        *response_len = 0;
        return STATUS_INVALID_DATA;
    }

    // Parse speed
    int16_t speed;
    memcpy(&speed, request_data, sizeof(int16_t));

    // Validate range
    if (speed < -1000 || speed > 1000) {
        *response_len = 0;
        return STATUS_INVALID_DATA;
    }

    // Apply speed
    stepper_set_speed(speed);

    // ACK response: just status byte
    response_data[0] = STATUS_OK;
    *response_len = 1;

    return STATUS_OK;
}

// Registration at initialization (stepper motor controller)
void app_init(void) {
    tsumikoro_bus_handle_t bus = tsumikoro_bus_init(&config);

    // Register stepper motor handlers (0x2000 range)
    tsumikoro_bus_register_command(bus, CMD_GET_POSITION,
                                   handle_get_position, "GET_POSITION");
    tsumikoro_bus_register_command(bus, CMD_SET_SPEED,
                                   handle_set_speed, "SET_SPEED");

    // Register generic handlers (0x0000 range) - all devices
    tsumikoro_bus_register_command(bus, 0x0001, handle_ping, "PING");
    tsumikoro_bus_register_command(bus, 0x0003, handle_stop, "STOP");
    // ... register other handlers
}
```

#### 4.7.4 Timeout Handling

**Controller Side** (sending commands):

```c
// Send command with timeout
tsumikoro_status_t tsumikoro_bus_send_command_blocking(
    tsumikoro_bus_handle_t bus,
    uint8_t dest_id,
    uint8_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len,
    uint32_t timeout_ms
) {
    // Send command packet
    tsumikoro_bus_send_packet(...);

    // Wait for response (or timeout)
    uint32_t start_time = get_time_ms();
    while ((get_time_ms() - start_time) < timeout_ms) {
        tsumikoro_packet_t response;
        if (tsumikoro_bus_receive_packet(bus, &response, 1) == STATUS_OK) {
            // Verify response matches command
            if (response.controller_id == dest_id && response.command == cmd) {
                // Extract response data
                memcpy(response_data, response.data, response.data_len);
                *response_len = response.data_len;

                // Check status byte
                if (response.data[0] == STATUS_OK) {
                    return TSUMIKORO_STATUS_OK;
                } else {
                    return TSUMIKORO_STATUS_COMMAND_FAILED;
                }
            }
        }
    }

    // Timeout expired
    return TSUMIKORO_STATUS_TIMEOUT;
}
```

**Peripheral Side** (receiving commands):

When a command packet is received, the peripheral:
1. Validates packet (CRC8, markers)
2. Looks up command handler in dispatch table
3. Calls handler with request data
4. Builds response packet with handler's output
5. Sends response packet
6. Returns to idle state

```c
// Internal command processing (called from RX callback)
void tsumikoro_bus_process_command(
    tsumikoro_bus_handle_t bus,
    const tsumikoro_packet_t *request
) {
    uint8_t response_data[TSUMIKORO_MAX_DATA_LEN];
    uint8_t response_len = 0;
    uint8_t status = STATUS_ERROR;

    // Find command handler
    tsumikoro_cmd_handler_t handler = find_command_handler(bus, request->command);

    if (handler != NULL) {
        // Call handler
        status = handler(
            request->command,
            request->data,
            request->data_len,
            response_data,
            &response_len
        );
    } else {
        // Unknown command
        response_data[0] = STATUS_INVALID_CMD;
        response_len = 1;
    }

    // Build and send response packet
    tsumikoro_packet_t response;
    response.start = TSUMIKORO_PACKET_START;
    response.controller_id = request->controller_id;  // Echo back
    response.command = request->command;              // Echo command
    response.data_len = response_len;
    memcpy(response.data, response_data, response_len);
    response.crc8 = tsumikoro_crc8(...);
    response.end = TSUMIKORO_PACKET_END;

    tsumikoro_bus_send_packet(bus, &response, 20);
}
```

#### 4.7.5 Broadcast Commands

Broadcast commands (dest_id = 0xFF) have special handling:
- **No response expected** from any device
- All devices process the command
- Useful for: emergency stop, sync commands, batch configuration
- No timeout or retry logic

**Example**: Emergency stop all motors
```c
// Controller sends broadcast STOP
tsumikoro_bus_send_command(bus, 0xFF, CMD_STEPPER_STOP, NULL, 0, 0);
// No response expected, command returns immediately

// All peripherals receive and execute, but don't respond
```

#### 4.7.6 Variable-Length Responses

Handlers can return variable amounts of data:

**Example**: Device information response
```c
uint8_t handle_get_info(
    uint8_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // Build device info structure
    typedef struct {
        uint8_t status;           // STATUS_OK
        uint8_t device_type;      // 0x01 = stepper, 0x02 = servo
        uint16_t firmware_version; // Major.Minor (0x0102 = v1.2)
        uint32_t serial_number;   // Unique serial
        char name[16];            // Device name (null-terminated)
    } __attribute__((packed)) device_info_t;

    device_info_t info = {
        .status = STATUS_OK,
        .device_type = 0x01,  // Stepper
        .firmware_version = 0x0100,  // v1.0
        .serial_number = 0x12345678,
        .name = "Stepper-001"
    };

    memcpy(response_data, &info, sizeof(info));
    *response_len = sizeof(info);  // Variable: 1+1+2+4+16 = 24 bytes

    return STATUS_OK;
}
```

#### 4.7.7 Retry Logic

**Automatic Retry with Exponential Backoff**:

```c
int retry_count = 0;
int backoff_ms = 5;  // Initial backoff
tsumikoro_status_t status;

while (retry_count < MAX_RETRIES) {
    status = tsumikoro_bus_send_command_blocking(
        bus, dest_id, cmd, req_data, req_len,
        resp_data, &resp_len, timeout_ms
    );

    if (status == TSUMIKORO_STATUS_OK) {
        break;  // Success
    }

    if (status == TSUMIKORO_STATUS_TIMEOUT) {
        // Exponential backoff before retry
        delay_ms(backoff_ms);
        backoff_ms *= 2;  // 5ms, 10ms, 20ms, ...
        retry_count++;
    } else {
        // Other errors (CRC, NAK) don't retry
        break;
    }
}

if (retry_count >= MAX_RETRIES) {
    // All retries exhausted
    return TSUMIKORO_STATUS_FAILED;
}
```

**Retry Policy**:
- **Timeout errors**: Retry with backoff (3 attempts)
- **CRC errors**: Don't retry (likely corruption, not missed response)
- **NAK errors**: Don't retry (command rejected by peripheral)
- **Busy status**: Retry immediately (1-2 attempts)

#### 4.7.8 Bus Scanning and Device Discovery

The controller (ESP32 bridge) can discover which devices are present on the bus by scanning all possible device IDs.

**Scan Methods**:

1. **Ping Sweep** (RECOMMENDED for controller-peripheral)
2. **Broadcast Discovery** (for peer-to-peer, requires collision handling)

##### Method 1: Ping Sweep

Controller iterates through all device IDs (0x01-0xFE) and sends CMD_PING to each:

```c
/**
 * @brief Scan bus for active devices
 * @param bus Bus handle
 * @param device_list Output buffer for discovered device IDs
 * @param max_devices Maximum number of devices to find
 * @param timeout_per_device Timeout for each ping (default: 20ms)
 * @return Number of devices found
 */
uint8_t tsumikoro_bus_scan(
    tsumikoro_bus_handle_t bus,
    uint8_t *device_list,
    uint8_t max_devices,
    uint32_t timeout_per_device
) {
    uint8_t found_count = 0;
    uint8_t response_data[32];
    uint8_t response_len;

    // Scan device IDs 0x01 through 0xFE
    for (uint16_t device_id = 0x01; device_id <= 0xFE && found_count < max_devices; device_id++) {
        // Send PING command (0x0001) to this device ID
        tsumikoro_status_t status = tsumikoro_bus_send_command_blocking(
            bus,
            (uint8_t)device_id,
            0x0001,  // CMD_PING
            NULL,    // No request data
            0,
            response_data,
            &response_len,
            timeout_per_device
        );

        if (status == TSUMIKORO_STATUS_OK && response_data[0] == STATUS_OK) {
            // Device responded - add to list
            device_list[found_count++] = (uint8_t)device_id;

            TSUMIKORO_LOG_DEBUG("Found device at ID 0x%02X", device_id);
        }
        // Timeout or error = no device at this ID, continue scanning
    }

    return found_count;
}
```

**Scan Timing**:
- 254 possible device IDs (0x01-0xFE)
- 20ms timeout per ID
- Total scan time: ~5 seconds (worst case, no devices)
- Actual time: `num_devices * 2ms + (254 - num_devices) * 20ms`

**Optimizations**:

1. **Reduce timeout**: Use 10ms timeout (faster scan, may miss slow devices)
2. **Scan ranges**: Only scan expected ranges (e.g., 0x01-0x10)
3. **Parallel polling**: Not possible on half-duplex bus
4. **Smart scan**: Remember last known device list, only re-check offline devices

**Example: Smart Scan**:

```c
typedef struct {
    uint8_t device_id;
    uint8_t device_type;
    uint32_t last_seen_ms;
    bool online;
} device_entry_t;

device_entry_t device_table[16];  // Track up to 16 devices
uint8_t device_count = 0;

void tsumikoro_bus_smart_scan(tsumikoro_bus_handle_t bus) {
    uint32_t now = get_time_ms();

    // First, re-check known devices
    for (uint8_t i = 0; i < device_count; i++) {
        uint8_t response[32];
        uint8_t response_len;

        tsumikoro_status_t status = tsumikoro_bus_send_command_blocking(
            bus, device_table[i].device_id, 0x0001, NULL, 0,
            response, &response_len, 10  // 10ms timeout
        );

        if (status == TSUMIKORO_STATUS_OK) {
            device_table[i].online = true;
            device_table[i].last_seen_ms = now;
        } else {
            device_table[i].online = false;
        }
    }

    // Periodically do full scan to discover new devices (every 60s)
    static uint32_t last_full_scan = 0;
    if ((now - last_full_scan) > 60000) {
        // Full scan logic here
        last_full_scan = now;
    }
}
```

##### Method 2: Broadcast Discovery (Peer-to-Peer)

Controller broadcasts a discovery request, all devices respond with random backoff:

```c
/**
 * @brief Broadcast discovery request
 * Devices respond with random backoff to avoid collisions
 */
void tsumikoro_bus_broadcast_discover(tsumikoro_bus_handle_t bus) {
    // Send broadcast IDENTIFY command (custom command 0xF001)
    tsumikoro_bus_send_command(
        bus,
        0xFF,     // Broadcast to all
        0xF001,   // CMD_IDENTIFY (custom)
        NULL,
        0,
        0         // No response expected for broadcast
    );

    // Listen for responses over next 500ms
    // Devices will respond with random backoff (0-500ms)
    uint32_t end_time = get_time_ms() + 500;

    while (get_time_ms() < end_time) {
        tsumikoro_packet_t response;
        if (tsumikoro_bus_receive_packet(bus, &response, 10) == STATUS_OK) {
            // Got a response - extract device info
            if (response.command == 0xF001) {
                uint8_t device_id = response.controller_id;
                add_device_to_list(device_id);
            }
        }
    }
}
```

**Device-side handler** (responds to broadcast discovery):

```c
// Handler for CMD_IDENTIFY (0xF001) - broadcast discovery
uint8_t handle_identify(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // Random backoff to avoid collisions (0-500ms)
    uint32_t backoff = rand() % 500;
    delay_ms(backoff);

    // Build device info response
    response_data[0] = STATUS_OK;
    response_data[1] = DEVICE_TYPE_STEPPER;  // Device type
    response_data[2] = (FIRMWARE_VERSION >> 8) & 0xFF;  // Version high
    response_data[3] = FIRMWARE_VERSION & 0xFF;          // Version low
    *response_len = 4;

    return STATUS_OK;
}
```

**Broadcast Discovery Issues**:
- Collisions possible even with random backoff
- Not deterministic
- Slower than ping sweep (need 500ms+ listen window)
- **Recommendation**: Use ping sweep for controller-peripheral, broadcast for peer-to-peer

##### Device Information Structure

When a device responds to CMD_PING or CMD_GET_INFO (0x0002), it returns:

```c
typedef struct {
    uint8_t status;              // STATUS_OK (0x00)
    uint8_t device_type;         // Device class
    uint16_t firmware_version;   // Version (0xMMmm = vM.m)
    uint32_t serial_number;      // Unique serial
    uint16_t capabilities;       // Capability flags
    char name[16];               // Device name (null-terminated)
} __attribute__((packed)) device_info_t;

// Device types (aligned with command ranges)
#define DEVICE_TYPE_UNKNOWN      0x00
#define DEVICE_TYPE_GENERIC      0x01
#define DEVICE_TYPE_SENSOR       0x10
#define DEVICE_TYPE_STEPPER      0x20
#define DEVICE_TYPE_SERVO        0x30
#define DEVICE_TYPE_DC_MOTOR     0x40
#define DEVICE_TYPE_BRIDGE       0x50

// Capability flags
#define CAPABILITY_BOOTLOADER    0x0001  // Supports bootloader mode
#define CAPABILITY_CONFIG_FLASH  0x0002  // Can save config to flash
#define CAPABILITY_OTA_UPDATE    0x0004  // Supports OTA updates
#define CAPABILITY_POSITION      0x0010  // Has position sensor
#define CAPABILITY_ENCODER       0x0020  // Has encoder
#define CAPABILITY_HOMING        0x0040  // Supports homing
```

##### CMD_SCAN Implementation (Bridge Command 0x5001)

The ESP32 bridge can expose a high-level scan command:

```c
// Bridge-specific command: Scan and return device list
uint8_t handle_scan(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    uint8_t device_list[64];  // Max 64 devices
    uint8_t timeout_ms = 20;  // Default timeout

    // Optional: Request data can specify timeout
    if (request_len == 1) {
        timeout_ms = request_data[0];
    }

    // Perform scan
    uint8_t found = tsumikoro_bus_scan(bus, device_list, 64, timeout_ms);

    // Build response: status + count + device IDs
    response_data[0] = STATUS_OK;
    response_data[1] = found;  // Number of devices found
    memcpy(&response_data[2], device_list, found);
    *response_len = 2 + found;

    return STATUS_OK;
}
```

**Response format**:

```
Response to CMD_SCAN (0x5001):
Byte 0: Status (0x00 = OK)
Byte 1: Device count (N)
Byte 2..N+1: Device IDs

Example (3 devices found at IDs 0x01, 0x05, 0x10):
00 03 01 05 10
│  │  └──┬──┘
│  │     └── Device IDs
│  └────── Count: 3 devices
└───────── Status: OK
```

##### Periodic Background Scanning

Controller can maintain a device presence table with periodic health checks:

```c
void tsumikoro_bus_background_task(void) {
    static uint32_t last_health_check = 0;
    uint32_t now = get_time_ms();

    // Health check every 10 seconds
    if ((now - last_health_check) > 10000) {
        // Ping all known devices
        for (uint8_t i = 0; i < device_count; i++) {
            uint8_t response[4];
            uint8_t response_len;

            tsumikoro_status_t status = tsumikoro_bus_send_command_blocking(
                bus, device_table[i].id, 0x0001, NULL, 0,
                response, &response_len, 10
            );

            if (status == TSUMIKORO_STATUS_OK) {
                device_table[i].online = true;
                device_table[i].last_seen_ms = now;
            } else if ((now - device_table[i].last_seen_ms) > 30000) {
                // No response for 30 seconds - mark offline
                device_table[i].online = false;
                TSUMIKORO_LOG_WARN("Device 0x%02X offline", device_table[i].id);
            }
        }

        last_health_check = now;
    }
}
```

##### Performance Considerations

**Full Bus Scan (254 IDs)**:

| Timeout | No devices | 10 devices | 50 devices |
|---------|------------|------------|------------|
| 5ms | 1.3s | 1.2s | 1.0s |
| 10ms | 2.5s | 2.4s | 2.0s |
| 20ms | 5.1s | 4.9s | 4.1s |

**Formula**: `scan_time = (254 - N) * timeout + N * response_time`
- N = number of devices
- response_time ≈ 2ms (at 1Mbaud)
- timeout = time to wait for non-existent device

**Recommendations**:
- **Initial scan**: Use 20ms timeout (thorough, 5s worst case)
- **Quick re-scan**: Use 10ms timeout (2.5s worst case)
- **Health check**: Only ping known devices (20ms per device)
- **Smart scan**: Limit range to expected IDs (e.g., 0x01-0x20 = 20 IDs * 20ms = 400ms)

##### Example: Complete Scan Function

```c
/**
 * @brief Comprehensive bus scan with device info
 */
typedef struct {
    uint8_t id;
    uint8_t type;
    uint16_t version;
    uint32_t serial;
    char name[16];
    bool online;
} scanned_device_t;

uint8_t tsumikoro_bus_scan_detailed(
    tsumikoro_bus_handle_t bus,
    scanned_device_t *devices,
    uint8_t max_devices
) {
    uint8_t found_count = 0;

    for (uint16_t id = 0x01; id <= 0xFE && found_count < max_devices; id++) {
        uint8_t response[32];
        uint8_t response_len;

        // Send CMD_GET_INFO (0x0002) to get detailed device info
        tsumikoro_status_t status = tsumikoro_bus_send_command_blocking(
            bus, (uint8_t)id, 0x0002, NULL, 0,
            response, &response_len, 20
        );

        if (status == TSUMIKORO_STATUS_OK && response[0] == STATUS_OK) {
            // Parse device info response
            devices[found_count].id = (uint8_t)id;
            devices[found_count].type = response[1];
            devices[found_count].version = (response[2] << 8) | response[3];
            memcpy(&devices[found_count].serial, &response[4], 4);
            memcpy(devices[found_count].name, &response[8], 16);
            devices[found_count].online = true;

            found_count++;

            TSUMIKORO_LOG_INFO("Found: ID=0x%02X Type=0x%02X Ver=%d.%d Name=%s",
                id, devices[found_count-1].type,
                response[2], response[3],
                devices[found_count-1].name
            );
        }
    }

    return found_count;
}
```

#### 4.7.9 Hardware-Assisted ID Assignment (OPTIONAL)

For applications requiring automatic ID assignment without manual configuration, an optional hardware daisy-chain signal can be used to sequentially assign device IDs.

**Feature Status**: OPTIONAL - Requires additional hardware GPIO connections

##### Hardware Setup

A dedicated GPIO signal is daisy-chained between peripherals:

```
Controller                  Peripheral #1           Peripheral #2           Peripheral #3
┌─────────┐                ┌──────────┐            ┌──────────┐            ┌──────────┐
│         │                │          │            │          │            │          │
│  ESP32  │  ID_CHAIN_OUT  │  STM32   │ ID_CHAIN   │  STM32   │ ID_CHAIN   │  STM32   │
│  Bridge ├───────────────>│ Motor #1 ├──────────>│ Motor #2 ├──────────>│ Motor #3 │
│         │   (GPIO HIGH)  │          │            │          │            │          │
│         │                │ ID_IN    │ ID_OUT     │ ID_IN    │ ID_OUT     │ ID_IN    │
│         │   RS-485 Bus   │          │            │          │            │          │
│         ├────────────────┤──────────┤────────────┤──────────┤────────────┤──────────┤
└─────────┘                └──────────┘            └──────────┘            └──────────┘
```

**Signal Characteristics**:
- **ID_CHAIN_OUT** (controller): GPIO output, set HIGH to start assignment
- **ID_IN** (peripheral): GPIO input, detects when this device should assign its ID
- **ID_OUT** (peripheral): GPIO output, passes signal to next device after assignment

**Default State**:
- All ID_OUT signals are LOW until device assigns its ID
- Controller ID_CHAIN_OUT is LOW during normal operation

##### Assignment Protocol

**Step-by-step sequence**:

1. Controller broadcasts `CMD_ID_ASSIGN_START` (0x000B)
2. All peripherals enter ID assignment mode, waiting for ID_IN = HIGH
3. Controller sets ID_CHAIN_OUT = HIGH
4. First peripheral (sees ID_IN = HIGH):
   - Assigns itself ID = 0x01
   - Saves ID to flash/EEPROM
   - Sets ID_OUT = HIGH (propagates to next device)
   - Sends acknowledgment to controller
5. Second peripheral (now sees ID_IN = HIGH):
   - Assigns itself ID = 0x02
   - Saves ID to flash/EEPROM
   - Sets ID_OUT = HIGH
   - Sends acknowledgment
6. Process continues down the chain
7. Controller waits for timeout (no more ACKs = assignment complete)
8. Controller broadcasts `CMD_ID_ASSIGN_COMPLETE` (0x000D)
9. All peripherals exit assignment mode, ID_OUT signals remain HIGH

##### Command Definitions

Add to Generic Commands (0x0000-0x0FFF):

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| CMD_ID_ASSIGN_START | 0x000B | Start hardware ID assignment | Broadcast (no response) |
| CMD_ID_ASSIGN_ACK | 0x000C | Peripheral acknowledges assigned ID | Controller receives ACK |
| CMD_ID_ASSIGN_COMPLETE | 0x000D | Assignment complete | Broadcast (no response) |

##### Controller-Side Implementation

```c
/**
 * @brief Perform hardware-assisted ID assignment
 * @param bus Bus handle
 * @param chain_out_gpio GPIO pin for ID_CHAIN_OUT signal
 * @param timeout_per_device Timeout to wait for each device (default: 100ms)
 * @return Number of devices assigned
 */
uint8_t tsumikoro_bus_hardware_id_assign(
    tsumikoro_bus_handle_t bus,
    gpio_num_t chain_out_gpio,
    uint32_t timeout_per_device
) {
    uint8_t assigned_count = 0;

    TSUMIKORO_LOG_INFO("Starting hardware ID assignment...");

    // Set ID_CHAIN_OUT to LOW initially
    gpio_set_level(chain_out_gpio, 0);
    delay_ms(10);

    // Broadcast ID_ASSIGN_START command
    tsumikoro_bus_send_command(
        bus,
        0xFF,           // Broadcast
        0x000B,         // CMD_ID_ASSIGN_START
        NULL,
        0,
        0
    );

    // Wait for all devices to enter assignment mode
    delay_ms(50);

    // Assert ID_CHAIN_OUT = HIGH to start assignment
    gpio_set_level(chain_out_gpio, 1);
    TSUMIKORO_LOG_DEBUG("ID_CHAIN_OUT asserted HIGH");

    // Listen for ACKs from each peripheral
    uint32_t last_ack_time = get_time_ms();

    while ((get_time_ms() - last_ack_time) < timeout_per_device) {
        tsumikoro_packet_t response;

        if (tsumikoro_bus_receive_packet(bus, &response, 10) == STATUS_OK) {
            if (response.command == 0x000C) {  // CMD_ID_ASSIGN_ACK
                uint8_t assigned_id = response.controller_id;
                assigned_count++;
                last_ack_time = get_time_ms();

                TSUMIKORO_LOG_INFO("Device assigned ID 0x%02X (%d devices total)",
                    assigned_id, assigned_count);
            }
        }
    }

    // Broadcast assignment complete
    tsumikoro_bus_send_command(
        bus,
        0xFF,           // Broadcast
        0x000D,         // CMD_ID_ASSIGN_COMPLETE
        NULL,
        0,
        0
    );

    TSUMIKORO_LOG_INFO("Hardware ID assignment complete: %d devices assigned",
        assigned_count);

    return assigned_count;
}
```

##### Peripheral-Side Implementation

```c
// Global state for ID assignment
static volatile bool id_assignment_mode = false;
static gpio_num_t id_in_gpio;
static gpio_num_t id_out_gpio;

/**
 * @brief Handler for CMD_ID_ASSIGN_START (0x000B)
 */
uint8_t handle_id_assign_start(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // Enter ID assignment mode
    id_assignment_mode = true;

    // Ensure ID_OUT is LOW initially
    gpio_set_level(id_out_gpio, 0);

    TSUMIKORO_LOG_INFO("Entered ID assignment mode");

    // Start background task to monitor ID_IN
    xTaskCreate(id_assignment_task, "id_assign", 2048, NULL, 5, NULL);

    return STATUS_OK;  // Broadcast, no response sent
}

/**
 * @brief Background task to monitor ID_IN signal
 */
void id_assignment_task(void *param) {
    uint8_t next_id = 0x01;

    while (id_assignment_mode) {
        // Check if ID_IN is HIGH
        if (gpio_get_level(id_in_gpio) == 1) {
            // This is our turn to assign ID

            // Calculate next available ID (could be 0x01, 0x02, etc.)
            // In simple case, we can count chain position or use stored value
            uint8_t my_id = next_id++;

            // Save ID to persistent storage
            save_device_id_to_flash(my_id);

            // Update runtime device ID
            g_device_id = my_id;

            TSUMIKORO_LOG_INFO("Assigned ID: 0x%02X", my_id);

            // Assert ID_OUT = HIGH to propagate to next device
            gpio_set_level(id_out_gpio, 1);

            // Send ACK to controller
            uint8_t ack_data[1] = { my_id };
            tsumikoro_bus_send_command(
                bus,
                0x00,       // To controller
                0x000C,     // CMD_ID_ASSIGN_ACK
                ack_data,
                1,
                20
            );

            // Exit task (assignment complete for this device)
            break;
        }

        delay_ms(10);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Handler for CMD_ID_ASSIGN_COMPLETE (0x000D)
 */
uint8_t handle_id_assign_complete(
    uint16_t cmd,
    const uint8_t *request_data,
    uint8_t request_len,
    uint8_t *response_data,
    uint8_t *response_len
) {
    // Exit ID assignment mode
    id_assignment_mode = false;

    TSUMIKORO_LOG_INFO("ID assignment complete, my ID: 0x%02X", g_device_id);

    return STATUS_OK;  // Broadcast, no response sent
}
```

##### Hardware Requirements

**Per Peripheral Device**:
- 2x GPIO pins (ID_IN, ID_OUT)
- Pull-down resistor on ID_IN (10kΩ recommended)
- No additional components required

**Controller**:
- 1x GPIO pin (ID_CHAIN_OUT)

**Wiring**:
- Use twisted pair or shielded cable for ID_CHAIN signal
- Keep signal length reasonable (<5m recommended)
- 3.3V or 5V logic levels (consistent across all devices)

##### Advantages

- **Zero configuration**: Peripherals automatically assigned sequential IDs
- **Physical order**: ID assignment follows physical cable order
- **Repeatable**: Consistent ID assignment after power cycle (stored in flash)
- **No conflicts**: Guaranteed unique IDs
- **Simple wiring**: Single additional signal wire

##### Limitations

- **Requires additional GPIO**: 2 pins per peripheral, 1 on controller
- **Physical wiring**: Daisy-chain must match desired ID order
- **Not hot-plug**: Assignment typically done during initial setup
- **Hardware dependency**: Not available on all platforms

##### Configuration

Enable in HAL platform config:

```c
// Platform-specific configuration
typedef struct {
    // ... existing fields ...

    // Optional: Hardware ID assignment
    bool enable_hw_id_assign;      // Enable hardware ID assignment feature
    gpio_num_t id_in_gpio;         // GPIO for ID_IN (peripheral only)
    gpio_num_t id_out_gpio;        // GPIO for ID_OUT (peripheral only)
    gpio_num_t id_chain_out_gpio;  // GPIO for ID_CHAIN_OUT (controller only)
} tsumikoro_platform_config_t;
```

##### Alternative: Simple Sequential Assignment

For simpler cases without daisy-chain hardware, peripherals can assign themselves based on their position in the response queue:

```c
// Controller broadcasts: "Who wants ID 0x01?"
// First peripheral to respond gets ID 0x01
// Controller broadcasts: "Who wants ID 0x02?"
// Next peripheral responds and gets ID 0x02
// ... continues until no more responses
```

This software-only approach requires no additional hardware but is less deterministic due to potential timing variations.

---

## 5. DMA Integration

### 5.1 RX DMA Strategy

**Circular Buffer with UART IDLE Detection**:

```
DMA Circular Buffer (256-8192 bytes, platform-dependent)
┌─────────────────────────────────────────────────┐
│ [Packet1] [Packet2] [Packet3] ... [Free Space] │
└─────────────────────────────────────────────────┘
           ↑                      ↑
         rx_tail               rx_head
        (app read)          (DMA write)
```

**Operation**:
1. DMA continuously writes incoming UART data to circular buffer
2. UART IDLE interrupt fires when bus goes idle (end of packet)
3. ISR calculates received bytes: `rx_count = rx_head - rx_tail`
4. Parse packet(s) from buffer without copying (zero-copy)
5. Update `rx_tail` after successful parsing
6. Handle buffer wraparound with modulo arithmetic

**Advantages**:
- Zero-copy parsing (parse directly from DMA buffer)
- No data loss from interrupt latency
- Automatic packet boundary detection via IDLE
- Efficient use of DMA peripheral

### 5.2 TX DMA Strategy

**Single Linear Buffer with Completion Interrupt**:

```
TX Buffer (70-256 bytes, platform-dependent)
┌────────────────────────────────────┐
│ [Packet to transmit] [Free]        │
└────────────────────────────────────┘
  ↑                    ↑
tx_start            tx_end
```

**Operation**:
1. Build packet in TX buffer (in-place construction)
2. Enable TX mode (set DE=1, RE=0 for RS-485)
3. Start DMA transfer from buffer to UART TX
4. DMA completion interrupt fires when done
5. Disable TX mode (set DE=0, RE=1)
6. Clear `tx_busy` flag
7. Optionally: Dequeue next packet from TX queue

**Optional: Multi-buffer Queuing**:
- Maintain array of TX buffers for queuing
- Double-buffering: Prepare next packet while transmitting current
- Increases throughput but uses more RAM

### 5.3 Platform-Specific Implementation

#### STM32 HAL

```c
// RX: Circular DMA
HAL_UART_Receive_DMA(&huart, rx_buffer, RX_BUFFER_SIZE);

// Enable UART IDLE interrupt
__HAL_UART_ENABLE_IT(&huart, UART_IT_IDLE);

// IDLE ISR
void UART_IDLE_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart);

        // Get DMA write position
        uint16_t dma_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart.hdmarx);

        // Calculate received bytes
        uint16_t rx_count = (dma_pos >= rx_tail) ?
                            (dma_pos - rx_tail) :
                            (RX_BUFFER_SIZE - rx_tail + dma_pos);

        // Parse packet(s)
        tsumikoro_bus_process_rx(rx_buffer, rx_tail, rx_count);

        // Update tail
        rx_tail = dma_pos;
    }
}

// TX: Normal DMA
HAL_UART_Transmit_DMA(&huart, tx_buffer, tx_length);

// TX Complete ISR
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Disable TX mode (DE=0, RE=1)
    HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

    // Clear busy flag
    tx_busy = false;

    // Signal completion to application
    tsumikoro_bus_tx_complete_callback();
}
```

#### ESP32 IDF

```c
// Install UART driver with DMA
uart_driver_install(UART_NUM,
                   RX_BUFFER_SIZE,
                   TX_BUFFER_SIZE,
                   10,  // Event queue size
                   &uart_queue,
                   0);  // No ISR flags

// RX: Event-based
uart_event_t event;
while (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
    if (event.type == UART_DATA) {
        // Read available data
        int len = uart_read_bytes(UART_NUM, rx_buffer, event.size, 0);

        // Parse packets
        tsumikoro_bus_process_rx(rx_buffer, 0, len);
    }
    else if (event.type == UART_PATTERN_DET) {
        // Optional: Use pattern detection for packet boundaries
    }
}

// TX: Blocking or async
uart_write_bytes(UART_NUM, tx_buffer, tx_length);
```

#### Host/Linux (POSIX termios)

```c
// Open serial port
int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

// Configure termios
struct termios tty;
tcgetattr(fd, &tty);

cfsetispeed(&tty, B1000000);  // 1Mbaud
cfsetospeed(&tty, B1000000);

tty.c_cflag |= (CLOCAL | CREAD);  // Enable receiver
tty.c_cflag &= ~PARENB;           // No parity
tty.c_cflag &= ~CSTOPB;           // 1 stop bit
tty.c_cflag &= ~CSIZE;
tty.c_cflag |= CS8;               // 8 data bits

tcsetattr(fd, TCSANOW, &tty);

// RX: Thread-based polling
void* rx_thread(void* arg) {
    uint8_t buffer[256];
    while (running) {
        int len = read(fd, buffer, sizeof(buffer));
        if (len > 0) {
            tsumikoro_bus_process_rx(buffer, 0, len);
        }
    }
}

// TX: Blocking write
write(fd, tx_buffer, tx_length);
```

### 5.4 Buffer Sizing

At **1Mbaud**, data arrives at ~100 bytes/ms. Buffer must accommodate:
- Multi-packet bursts
- ISR latency (context switch, higher priority interrupts)
- Packet parsing time

| Platform | RX Buffer | TX Buffer | Queue Depth | Rationale |
|----------|-----------|-----------|-------------|-----------|
| STM32G030 | 256B | 140B | 2 packets | Limited RAM (8KB total) |
| STM32G071 | 1KB | 256B | 8 packets | Comfortable headroom |
| ESP32-S3 | 4KB | 512B | 32 packets | Large buffers for throughput |
| Host/Linux | 8KB | 1KB | 64 packets | Testing flexibility |

**Calculation Example (STM32G030)**:
- Max packet size: 70 bytes
- ISR latency: ~100µs = ~10 bytes at 1Mbaud
- Safety margin: 2x
- Min RX buffer: 70 + 10 = 80 bytes
- Recommended: 256 bytes (3+ packets)

---

## 6. Hardware Abstraction Layer (HAL)

### 6.1 HAL Interface

The HAL provides a platform-independent interface for UART and DMA operations:

```c
// hal/tsumikoro_bus_hal.h

/**
 * @brief HAL initialization configuration
 */
typedef struct {
    uint32_t baud_rate;        // Baud rate (typically 1000000)
    uint8_t *rx_buffer;        // RX DMA buffer (circular)
    size_t rx_buffer_size;     // RX buffer size
    uint8_t *tx_buffer;        // TX DMA buffer
    size_t tx_buffer_size;     // TX buffer size
    void *platform_config;     // Platform-specific config
} tsumikoro_hal_config_t;

/**
 * @brief Initialize HAL layer
 * @param config Configuration structure
 * @return 0 on success, error code otherwise
 */
int tsumikoro_hal_init(const tsumikoro_hal_config_t *config);

/**
 * @brief Deinitialize HAL layer
 */
void tsumikoro_hal_deinit(void);

/**
 * @brief Start RX DMA transfer (circular mode)
 * @param buffer RX buffer pointer
 * @param size Buffer size
 * @return 0 on success
 */
int tsumikoro_hal_start_rx_dma(uint8_t *buffer, size_t size);

/**
 * @brief Start TX DMA transfer (normal mode)
 * @param buffer TX buffer pointer
 * @param size Number of bytes to transmit
 * @return 0 on success
 */
int tsumikoro_hal_start_tx_dma(const uint8_t *buffer, size_t size);

/**
 * @brief Stop RX DMA transfer
 */
void tsumikoro_hal_stop_rx_dma(void);

/**
 * @brief Stop TX DMA transfer
 */
void tsumikoro_hal_stop_tx_dma(void);

/**
 * @brief Get current RX DMA write position
 * @return Number of bytes written by DMA
 */
size_t tsumikoro_hal_get_rx_position(void);

/**
 * @brief Check if TX DMA transfer is complete
 * @return true if complete
 */
bool tsumikoro_hal_is_tx_complete(void);

/**
 * @brief Check if bus is idle (no RX activity)
 * @return true if idle
 */
bool tsumikoro_hal_is_bus_idle(void);

/**
 * @brief Enable TX mode (half-duplex control)
 * Sets DE=1, /RE=1 for RS-485
 */
void tsumikoro_hal_enable_tx(void);

/**
 * @brief Enable RX mode (half-duplex control)
 * Sets DE=0, /RE=0 for RS-485
 */
void tsumikoro_hal_enable_rx(void);

// Callbacks (weak symbols, implemented by bus layer)

/**
 * @brief Called when UART IDLE detected (end of packet)
 */
void tsumikoro_hal_rx_idle_callback(void);

/**
 * @brief Called when TX DMA transfer completes
 */
void tsumikoro_hal_tx_complete_callback(void);

/**
 * @brief Called on UART error
 * @param error_code Platform-specific error code
 */
void tsumikoro_hal_error_callback(uint32_t error_code);
```

### 6.2 Platform-Specific Configs

#### STM32 Config

```c
// hal/stm32/tsumikoro_bus_hal_stm32.h

typedef struct {
    UART_HandleTypeDef *huart;  // STM32 HAL UART handle
    GPIO_TypeDef *de_port;      // DE pin GPIO port (RS-485)
    uint16_t de_pin;            // DE pin number
} tsumikoro_hal_stm32_config_t;
```

#### ESP32 Config

```c
// hal/esp32/tsumikoro_bus_hal_esp32.h

typedef struct {
    uart_port_t uart_num;       // UART number (0, 1, 2)
    int tx_pin;                 // TX GPIO pin
    int rx_pin;                 // RX GPIO pin
    int de_pin;                 // DE pin (RS-485, -1 if unused)
    QueueHandle_t *event_queue; // UART event queue
} tsumikoro_hal_esp32_config_t;
```

#### Host/Linux Config

```c
// hal/host/tsumikoro_bus_hal_host.h

typedef struct {
    const char *device_path;    // e.g., "/dev/ttyUSB0"
    int fd;                      // File descriptor
    pthread_t rx_thread;         // RX polling thread
    bool use_pty;                // Use pseudoterminal for loopback
} tsumikoro_hal_host_config_t;
```

#### Mock Config (Unit Testing)

```c
// hal/mock/tsumikoro_bus_hal_mock.h

typedef struct {
    uint8_t *virtual_rx_data;    // Simulated RX data
    size_t virtual_rx_len;       // RX data length
    uint8_t *captured_tx_data;   // Captured TX data
    size_t captured_tx_len;      // TX data length
    bool simulate_errors;        // Enable error injection
    uint32_t latency_us;         // Simulated transmission delay
} tsumikoro_hal_mock_config_t;

// Test helpers
void tsumikoro_hal_mock_inject_rx(const uint8_t *data, size_t len);
void tsumikoro_hal_mock_get_tx(uint8_t *buffer, size_t *len);
void tsumikoro_hal_mock_trigger_idle(void);
void tsumikoro_hal_mock_simulate_error(int error_type);
```

### 6.3 Host/Linux HAL Implementation

Uses POSIX termios for serial port access, enabling:
- Real serial port communication (`/dev/ttyUSB0`, `/dev/ttyACM0`)
- Virtual serial port pairs (`socat`, `pty`) for loopback testing
- Thread-based "DMA" simulation using `pthread`
- Compatible with Linux, macOS, BSD

**Key Features**:
- Non-blocking I/O with `select()` or `poll()`
- Configurable baud rate via `cfsetispeed()/cfsetospeed()`
- Thread-based RX for async operation
- Direct `read()/write()` for TX

### 6.4 Mock HAL for Pure Unit Tests

Fully simulated HAL with no hardware dependencies:
- In-memory RX/TX buffers
- Deterministic behavior for CI/CD
- Fast test execution (no hardware delays)
- Error injection for robustness testing

**Test Features**:
- Inject RX data programmatically
- Capture TX data for verification
- Trigger IDLE interrupt manually
- Simulate errors (timeouts, CRC errors, overruns)
- Configurable latency simulation

---

## 7. API Design

### 7.1 Initialization API

```c
/**
 * @brief Bus configuration structure
 */
typedef struct {
    uint8_t device_id;           // This device's ID (0x00-0xFF)
    uint32_t baud_rate;          // Baud rate (typically 1000000)
    uint16_t rx_buffer_size;     // RX buffer size (platform-dependent)
    uint16_t tx_queue_depth;     // TX queue depth
    bool is_controller;          // True if controller mode
    void *platform_config;       // Platform-specific HAL config
} tsumikoro_bus_config_t;

/**
 * @brief Opaque bus handle
 */
typedef struct tsumikoro_bus_context* tsumikoro_bus_handle_t;

/**
 * @brief Initialize bus library
 * @param config Configuration structure
 * @return Bus handle, or NULL on error
 */
tsumikoro_bus_handle_t tsumikoro_bus_init(const tsumikoro_bus_config_t *config);

/**
 * @brief Deinitialize bus library
 * @param bus Bus handle
 */
void tsumikoro_bus_deinit(tsumikoro_bus_handle_t bus);
```

### 7.2 Transmit API

```c
/**
 * @brief Send raw packet (advanced users)
 * @param bus Bus handle
 * @param packet Pointer to packet structure
 * @param timeout_ms Timeout in milliseconds
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_send_packet(
    tsumikoro_bus_handle_t bus,
    const tsumikoro_packet_t *packet,
    uint32_t timeout_ms
);

/**
 * @brief Send command (convenience function)
 * @param bus Bus handle
 * @param dest_id Destination device ID (or 0xFF for broadcast)
 * @param cmd Command word (16-bit)
 * @param data Data payload
 * @param data_len Data length (0-64)
 * @param timeout_ms Timeout in milliseconds
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_send_command(
    tsumikoro_bus_handle_t bus,
    uint8_t dest_id,
    uint16_t cmd,
    const uint8_t *data,
    uint8_t data_len,
    uint32_t timeout_ms
);
```

### 7.3 Receive API

```c
/**
 * @brief RX callback function type
 * @param packet Received packet
 * @param user_data User-provided context
 */
typedef void (*tsumikoro_rx_callback_t)(
    const tsumikoro_packet_t *packet,
    void *user_data
);

/**
 * @brief Register callback for received packets
 * @param bus Bus handle
 * @param cmd Command to filter (or 0xFFFF for all)
 * @param callback Callback function
 * @param user_data User context passed to callback
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_register_callback(
    tsumikoro_bus_handle_t bus,
    uint16_t cmd,
    tsumikoro_rx_callback_t callback,
    void *user_data
);

/**
 * @brief Unregister callback
 * @param bus Bus handle
 * @param cmd Command to unregister
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_unregister_callback(
    tsumikoro_bus_handle_t bus,
    uint16_t cmd
);

/**
 * @brief Poll for received packets (non-callback mode)
 * @param bus Bus handle
 * @param timeout_ms Timeout in milliseconds
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_poll(
    tsumikoro_bus_handle_t bus,
    uint32_t timeout_ms
);

/**
 * @brief Receive packet (blocking, non-callback mode)
 * @param bus Bus handle
 * @param packet Buffer for received packet
 * @param timeout_ms Timeout in milliseconds
 * @return Status code
 */
tsumikoro_status_t tsumikoro_bus_receive_packet(
    tsumikoro_bus_handle_t bus,
    tsumikoro_packet_t *packet,
    uint32_t timeout_ms
);
```

### 7.4 Utility API

```c
/**
 * @brief Check if bus is busy transmitting
 * @param bus Bus handle
 * @return true if busy
 */
bool tsumikoro_bus_is_busy(tsumikoro_bus_handle_t bus);

/**
 * @brief Flush TX queue and RX buffers
 * @param bus Bus handle
 */
void tsumikoro_bus_flush(tsumikoro_bus_handle_t bus);

/**
 * @brief Bus statistics structure
 */
typedef struct {
    uint32_t tx_packets;         // Transmitted packets
    uint32_t rx_packets;         // Received packets
    uint32_t tx_errors;          // TX errors
    uint32_t rx_errors;          // RX errors
    uint32_t crc_errors;         // CRC validation failures
    uint32_t timeouts;           // Timeout events
    uint32_t retries;            // Retry attempts
    uint32_t bus_collisions;     // Collisions (peer-to-peer mode)
} tsumikoro_bus_stats_t;

/**
 * @brief Get bus statistics
 * @param bus Bus handle
 * @param stats Pointer to stats structure
 */
void tsumikoro_bus_get_stats(
    tsumikoro_bus_handle_t bus,
    tsumikoro_bus_stats_t *stats
);

/**
 * @brief Reset statistics counters
 * @param bus Bus handle
 */
void tsumikoro_bus_reset_stats(tsumikoro_bus_handle_t bus);
```

---

## 8. State Machine

### 8.1 Bus States

```
              ┌─────────┐
              │  IDLE   │
              └────┬────┘
                   │
        ┌──────────┼──────────┐
        │                     │
        ▼                     ▼
   ┌─────────┐         ┌──────────┐
   │TX_PREPARE│         │RX_IDLE   │
   └────┬─────┘         │ _DETECT  │
        │               └────┬─────┘
        ▼                    │
   ┌─────────────┐           ▼
   │TX_TRANSMIT  │      ┌────────────┐
   └────┬────────┘      │RX_RECEIVING│
        │               └─────┬──────┘
        ▼                     │
   ┌──────────────┐           ▼
   │TX_WAIT_COMPLETE│    ┌────────────┐
   └────┬─────────┘      │RX_PROCESSING│
        │                └─────┬──────┘
        │                      │
        └──────────┬───────────┘
                   │
                   ▼
              ┌─────────┐
              │  IDLE   │
              └─────────┘
```

### 8.2 State Transitions

| Current State | Event | Next State | Action |
|---------------|-------|------------|--------|
| IDLE | TX request | TX_PREPARE | Build packet in TX buffer |
| TX_PREPARE | Packet ready | TX_TRANSMIT | Enable TX mode, start DMA |
| TX_TRANSMIT | DMA complete | TX_WAIT_COMPLETE | Wait for UART shift register empty |
| TX_WAIT_COMPLETE | TX complete | IDLE | Disable TX mode, enable RX |
| IDLE | RX data | RX_IDLE_DETECT | Wait for IDLE interrupt |
| RX_IDLE_DETECT | IDLE IRQ | RX_RECEIVING | Calculate RX count |
| RX_RECEIVING | Data ready | RX_PROCESSING | Parse packet(s) |
| RX_PROCESSING | Parse complete | IDLE | Invoke callbacks |
| Any | Error | ERROR | Handle error, return to IDLE |

### 8.3 Error States

| Error | Recovery Action |
|-------|----------------|
| TIMEOUT | Increment timeout counter, retry or abort |
| CRC_ERROR | Discard packet, increment CRC error counter |
| DMA_OVERRUN | Flush RX buffer, reset DMA, increment error counter |
| TX_COLLISION | Random backoff (peer-to-peer mode), retry |
| INVALID_LENGTH | Discard packet, resync to next START marker |

---

## 9. Error Handling & Recovery

### 9.1 Error Detection

| Error Type | Detection Method | Priority |
|------------|------------------|----------|
| CRC mismatch | CRC8 validation fails | High |
| Timeout | No response within timeout window | Medium |
| Framing error | UART reports framing error | High |
| Overrun | DMA buffer full, data lost | Critical |
| Invalid length | data_len > MAX_DATA_LEN | Medium |
| Invalid markers | start != 0xAA or end != 0x55 | Medium |

### 9.2 Recovery Strategies

**Automatic Retry with Exponential Backoff**:

```c
int retry_count = 0;
int backoff_ms = INITIAL_BACKOFF;  // e.g., 5ms

while (retry_count < MAX_RETRIES) {
    status = tsumikoro_bus_send_packet(bus, packet, timeout);

    if (status == TSUMIKORO_STATUS_OK) {
        break;  // Success
    }

    // Exponential backoff
    delay_ms(backoff_ms);
    backoff_ms *= 2;  // 5ms, 10ms, 20ms, ...
    retry_count++;
}

if (retry_count >= MAX_RETRIES) {
    // Report failure to application
    tsumikoro_bus_error_callback(TSUMIKORO_ERROR_MAX_RETRIES);
}
```

**Bus Reset**:
- After repeated failures (e.g., 10 consecutive errors)
- Flush all buffers
- Reset DMA
- Reinitialize UART
- Clear error counters
- Notify application via callback

### 9.3 Error Callbacks

```c
/**
 * @brief Error callback function type
 * @param error_code Error code
 * @param context Error context (packet, device ID, etc.)
 */
typedef void (*tsumikoro_error_callback_t)(
    tsumikoro_error_t error_code,
    void *context
);

/**
 * @brief Register error callback
 * @param bus Bus handle
 * @param callback Callback function
 */
void tsumikoro_bus_register_error_callback(
    tsumikoro_bus_handle_t bus,
    tsumikoro_error_callback_t callback
);
```

### 9.4 CRC8 Implementation

**Table-Based Implementation** (recommended for performance):

```c
// CRC-8-CCITT lookup table (polynomial 0x07)
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/**
 * @brief Calculate CRC8-CCITT checksum (table-based)
 *
 * Polynomial: 0x07 (x^8 + x^2 + x + 1)
 * Init: 0x00, Final XOR: 0x00
 *
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return CRC8 checksum
 */
static inline uint8_t tsumikoro_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
```

**Bitwise Implementation** (for size-constrained builds):

```c
/**
 * @brief Calculate CRC8-CCITT (bitwise, slower but smaller code)
 *
 * Use on STM32G030 to save 256 bytes of Flash
 */
static inline uint8_t tsumikoro_crc8_bitwise(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  // CRC-8-CCITT polynomial
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
```

**Packet Validation**:

```c
/**
 * @brief Validate packet CRC8 and markers
 *
 * @param packet Pointer to received packet
 * @return true if packet is valid
 */
static inline bool tsumikoro_validate_packet(const tsumikoro_packet_t *packet) {
    // Check start/end markers
    if (packet->start != TSUMIKORO_PACKET_START) return false;
    if (packet->end != TSUMIKORO_PACKET_END) return false;

    // Check data length
    if (packet->data_len > TSUMIKORO_MAX_DATA_LEN) return false;

    // Calculate CRC8 over: ID + CMD + LEN + DATA
    uint8_t calc_crc = tsumikoro_crc8(
        (const uint8_t *)&packet->controller_id,
        3 + packet->data_len  /* ID + CMD + LEN + DATA */
    );

    return (calc_crc == packet->crc8);
}
```

### 9.5 Diagnostics

```c
/**
 * @brief Get last error code
 * @param bus Bus handle
 * @return Last error code
 */
tsumikoro_error_t tsumikoro_bus_get_last_error(tsumikoro_bus_handle_t bus);

/**
 * @brief Enable debug logging (compile-time option)
 */
#if TSUMIKORO_BUS_ENABLE_DEBUG
    #define TSUMIKORO_LOG_DEBUG(fmt, ...) printf("[TSUMIKORO] " fmt "\n", ##__VA_ARGS__)
#else
    #define TSUMIKORO_LOG_DEBUG(fmt, ...)
#endif
```

---

## 10. Memory Management

### 10.1 Static Allocation

All memory is statically allocated at initialization:
- No `malloc()`/`free()` calls (embedded safety)
- Buffers allocated from `.bss` section or provided by user
- Configurable sizes via `tsumikoro_bus_config.h`

**Rationale**:
- Deterministic memory usage
- No heap fragmentation
- Suitable for safety-critical applications
- Easier certification (MISRA-C compliance)

### 10.2 Memory Footprint

| Component | STM32G030 | STM32G071 | ESP32-S3 |
|-----------|-----------|-----------|----------|
| RX Buffer | 256B | 1KB | 4KB |
| TX Buffer | 140B | 256B | 512B |
| TX Queue | 140B (2 pkts) | 1KB (8 pkts) | 4KB (32 pkts) |
| Context | ~100B | ~100B | ~100B |
| **Total** | **~636B** | **~2.4KB** | **~8.6KB** |

**Code Size** (approximate):
- Core library: ~3-4KB
- HAL (STM32): ~1-2KB
- HAL (ESP32): ~1KB (uses IDF UART driver)
- CRC8 table: 256B (or 0B if bitwise)
- **Total**: ~5-7KB

### 10.3 Zero-Copy Optimization

Parse packets directly from DMA RX buffer without copying:

```c
// Traditional approach (copies data)
uint8_t temp_buffer[70];
memcpy(temp_buffer, &rx_buffer[rx_tail], packet_len);
parse_packet(temp_buffer);

// Zero-copy approach (no memcpy)
tsumikoro_packet_t *packet = (tsumikoro_packet_t *)&rx_buffer[rx_tail];
if (tsumikoro_validate_packet(packet)) {
    process_packet(packet);
}
```

**Benefits**:
- Saves ~70 bytes of stack/RAM
- Faster execution (~100 cycles saved)
- Lower CPU overhead

---

## 11. Thread Safety & Concurrency

### 11.1 STM32 (Bare Metal)

**Interrupt Safety**:
- Critical sections protected with `__disable_irq()` / `__enable_irq()`
- TX queue uses atomic flags (`volatile bool tx_busy`)
- RX callbacks run in interrupt context (keep short!)

**Example**:
```c
void tsumikoro_bus_send_packet(...) {
    __disable_irq();
    if (tx_busy) {
        __enable_irq();
        return TSUMIKORO_STATUS_BUSY;
    }
    tx_busy = true;
    __enable_irq();

    // Build and send packet
}
```

### 11.2 ESP32 (FreeRTOS)

**Thread Safety**:
- Mutex protection for API calls
- ISR-safe queue operations (`xQueueSendFromISR()`)
- Callbacks can run in ISR or task context (configurable)

**Example**:
```c
void tsumikoro_bus_send_packet(...) {
    xSemaphoreTake(bus_mutex, portMAX_DELAY);

    // Build and send packet

    xSemaphoreGive(bus_mutex);
}
```

### 11.3 Host/Linux (POSIX)

**Thread Safety**:
- Mutex protection (`pthread_mutex_t`)
- RX thread runs independently
- Thread-safe queue for RX packets

---

## 12. Testing Strategy

### 12.1 Unit Tests (Host Platform)

Run on host PC/Linux without hardware using Mock HAL.

**Test Framework**: Simple assert-based or Unity/CppUTest

**Test Files**:

#### `tests/test_crc8.c`

```c
void test_crc8_known_vectors(void);      // CRC-8-CCITT test vectors
void test_crc8_single_bit_errors(void);  // Detect all 1-bit flips
void test_crc8_burst_errors(void);       // Detect burst errors
void test_crc8_table_vs_bitwise(void);   // Verify equivalence
```

#### `tests/test_protocol.c`

```c
void test_packet_build(void);            // Build valid packets
void test_packet_parse(void);            // Parse valid packets
void test_packet_validation(void);       // Reject invalid packets
void test_broadcast_address(void);       // Broadcast handling
```

#### `tests/test_state_machine.c`

```c
void test_idle_to_tx_transition(void);   // TX state flow
void test_idle_to_rx_transition(void);   // RX state flow
void test_timeout_handling(void);        // Timeout recovery
void test_error_states(void);            // Error transitions
```

#### `tests/test_dma_buffers.c`

```c
void test_circular_buffer_wraparound(void); // Handle buffer wrap
void test_zero_copy_parsing(void);          // Parse without copy
void test_buffer_overrun(void);             // Detect overrun
```

#### `tests/test_integration.c`

```c
void test_loopback(void);                   // TX → RX on same device
void test_multi_device(void);               // Simulate 3+ devices
void test_error_injection(void);            // Corrupt packets
void test_master_slave_roundtrip(void);     // Full transaction
```

**Running Tests**:
```bash
cd firmware/shared/tsumikoro_bus
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make
ctest --verbose
```

### 12.2 Integration Tests (Mock HAL)

Simulate multi-device bus without hardware:

```c
// Example: Controller-peripheral roundtrip
void test_controller_peripheral_roundtrip(void) {
    // Create virtual devices
    tsumikoro_bus_handle_t controller = create_mock_bus(0x00, true);
    tsumikoro_bus_handle_t peripheral = create_mock_bus(0x01, false);

    // Connect them via mock layer
    tsumikoro_mock_connect(controller, peripheral);

    // Controller sends GET_STATUS command
    tsumikoro_bus_send_command(controller, 0x01, CMD_GET_STATUS, NULL, 0, 20);

    // Process on peripheral
    tsumikoro_mock_process(peripheral);

    // Peripheral responds (simulated in test)
    uint8_t status_data[8] = { /* status payload */ };
    tsumikoro_bus_send_command(peripheral, 0x00, CMD_GET_STATUS, status_data, 8, 20);

    // Process on controller
    tsumikoro_mock_process(controller);

    // Verify response received
    assert_response_received();
}
```

### 12.3 Hardware Tests (Real Devices)

**Test Bench**:
- ESP32-S3 bridge (controller) on `/dev/ttyUSB0`
- STM32G071 motor controller (peripheral #1)
- STM32G030 motor controller (peripheral #2)
- RS-485 transceiver (MAX485 or equivalent)
- Logic analyzer on bus lines (A, B, DE)
- Oscilloscope for signal integrity

**Test Cases**:
1. Baud rate accuracy (verify actual 1Mbaud with scope)
2. Signal integrity (rise/fall times, voltage levels)
3. Multi-drop addressing (send to specific device ID)
4. Broadcast (all devices respond to 0xFF)
5. Sustained throughput (hours of operation)
6. Error rate measurement (inject noise)
7. Temperature stress testing (-40°C to +85°C)
8. Cable length testing (1m, 10m, 100m)

### 12.4 Continuous Integration

**GitHub Actions Workflow**:

```yaml
name: Test Tsumikoro Bus

on: [push, pull_request]

jobs:
  test-bus-library:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: sudo apt-get install -y cmake build-essential

      - name: Build and test
        run: |
          cd firmware/shared/tsumikoro_bus
          mkdir build && cd build
          cmake .. -DBUILD_TESTS=ON
          make
          ctest --output-on-failure

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-results
          path: firmware/shared/tsumikoro_bus/build/Testing/
```

**Benefits**:
- Automated testing on every commit
- Catch regressions early
- No hardware required
- Fast feedback (<1 minute)

---

## 13. Performance Considerations

### 13.1 Latency Budget (1Mbaud, max packet 70 bytes)

| Phase | Time | Notes |
|-------|------|-------|
| Byte time | ~10µs | 1 / 1,000,000 * 10 bits |
| Packet TX time | ~700µs | 70 bytes * 10µs |
| DMA overhead | ~50µs | Setup and teardown |
| CRC8 calculation | ~4µs (table) | 70 bytes * 4 cycles @ 64MHz |
| Processing | <500µs | Parsing, callbacks |
| **Total latency** | **<2ms** | Command → Response |

**Comparison with 115200 baud**:
- Packet TX: 700µs vs 6ms (8.6x faster)
- Total latency: <2ms vs <10ms (5x faster)

### 13.2 Throughput

**Theoretical Maximum** (half-duplex):
- Raw baud rate: 1Mbaud = 1,000,000 bits/sec = 100,000 bytes/sec
- UART overhead: 2 bits/byte (start + stop) = 20% overhead
- Effective: ~83,000 bytes/sec

**Actual Throughput** (with protocol overhead):
- Packet overhead: 6 bytes (START, ID, CMD, LEN, CRC8, END)
- Average payload: 32 bytes
- Overhead ratio: 6 / (6 + 32) = 15.8%
- Effective: ~70,000-80,000 bytes/sec

**Multi-Drop Polling Example** (10 devices):
- Transaction per device: ~2ms
- Polling cycle: ~20ms
- Polling rate: 50Hz
- Throughput per device: ~3.5KB/s

### 13.3 CPU Usage

**DMA Mode** (recommended):
- Interrupt rate: ~1.4kHz (one per packet at max rate)
- ISR execution: ~50µs (parsing + callback)
- CPU time: 50µs * 1400/sec = 70ms/sec = **7% worst case**
- Typical: **<2%** (lower packet rate)

**Polling Mode** (not recommended):
- Continuous polling overhead: ~10% at 100Hz
- Use only for debugging or non-critical applications

**CRC8 Overhead**:

| Platform | Implementation | Calculation Time | % of TX Time |
|----------|----------------|------------------|--------------|
| STM32G030 (64MHz) | Bitwise | ~44µs | 6.3% |
| STM32G071 (64MHz) | Table | ~4.4µs | 0.6% |
| ESP32-S3 (240MHz) | Table | ~1.2µs | 0.2% |

### 13.4 STM32G030 Viability at 1Mbaud

The smallest target (8KB RAM, 64MHz, 32KB Flash) can handle 1Mbaud:

**UART Clock**:
- HSI16 (16MHz internal oscillator) can achieve 1Mbaud
- Prescaler: 16MHz / 16 = 1MHz ✓

**DMA Bandwidth**:
- STM32G0 DMA can handle 1Mbaud easily
- DMA max transfer rate: ~10MB/s >> 100KB/s required

**Interrupt Rate**:
- Worst case: 1400 interrupts/sec (one per packet)
- ISR overhead: 50µs per interrupt
- Total: 70ms/sec = **7% CPU** (acceptable)

**Recommendation**: Use DMA mode. Polling mode too slow on G030.

---

## 14. Configuration Options

```c
// tsumikoro_bus_config.h

// Platform selection (auto-detected or override)
#if defined(STM32G030) || defined(STM32G071)
    #define TSUMIKORO_PLATFORM_STM32
    #define TSUMIKORO_BUS_RX_BUFFER_SIZE    256
    #define TSUMIKORO_BUS_TX_QUEUE_DEPTH    2

    #if defined(STM32G030)
        // Use bitwise CRC to save 256 bytes Flash
        #define TSUMIKORO_CRC8_USE_TABLE    0
    #else
        // Use table for better performance
        #define TSUMIKORO_CRC8_USE_TABLE    1
    #endif

#elif defined(ESP32) || defined(ESP32S3)
    #define TSUMIKORO_PLATFORM_ESP32
    #define TSUMIKORO_BUS_RX_BUFFER_SIZE    4096
    #define TSUMIKORO_BUS_TX_QUEUE_DEPTH    32
    #define TSUMIKORO_CRC8_USE_TABLE        1

#elif defined(__linux__) || defined(__APPLE__)
    #define TSUMIKORO_PLATFORM_HOST
    #define TSUMIKORO_BUS_RX_BUFFER_SIZE    8192
    #define TSUMIKORO_BUS_TX_QUEUE_DEPTH    64
    #define TSUMIKORO_CRC8_USE_TABLE        1

#elif defined(TSUMIKORO_MOCK_HAL)
    #define TSUMIKORO_PLATFORM_MOCK
    #define TSUMIKORO_BUS_RX_BUFFER_SIZE    1024
    #define TSUMIKORO_BUS_TX_QUEUE_DEPTH    16
    #define TSUMIKORO_CRC8_USE_TABLE        1

#else
    #error "Unknown platform. Define: STM32, ESP32, HOST, or MOCK"
#endif

// Baud rate
#define TSUMIKORO_BUS_BAUD_RATE         1000000  // 1Mbaud

// Timing (tuned for 1Mbaud)
#define TSUMIKORO_BUS_MAX_RETRIES       3
#define TSUMIKORO_BUS_TIMEOUT_MS        20       // Response timeout
#define TSUMIKORO_BUS_BYTE_TIMEOUT_MS   1        // Inter-byte timeout

// Features
#define TSUMIKORO_BUS_ENABLE_STATS      1        // 0 to save ~100B RAM
#define TSUMIKORO_BUS_ENABLE_DEBUG      0        // 1 for verbose logging
#define TSUMIKORO_BUS_USE_DMA           1        // 0 for interrupt mode

// CRC wrapper macro
#if TSUMIKORO_CRC8_USE_TABLE
    #define tsumikoro_calculate_crc(data, len) tsumikoro_crc8(data, len)
#else
    #define tsumikoro_calculate_crc(data, len) tsumikoro_crc8_bitwise(data, len)
#endif
```

---

## 15. Future Enhancements

### 15.1 Protocol Enhancements

- **CRC-32** for very long packets or safety-critical commands
- **Protocol version negotiation** for mixed CRC8/CRC16 deployments
- **Dual CRC** (CRC8 + CRC16) for safety-critical applications
- **Sequence numbers** for detecting lost packets
- **Acknowledgments** (ACK/NAK) for reliable delivery

### 15.2 Performance Enhancements

- **Higher baud rates**: 2Mbaud, 4Mbaud (ESP32 supports up to 5Mbaud)
- **Hardware CRC** on STM32 (if available in custom CRC mode)
- **Zero-copy TX** (build packet directly in DMA buffer)
- **DMA chaining** for multi-packet bursts

### 15.3 Topology Enhancements

- **Automatic device discovery** (controller scans for peripherals)
- **Hot-plug support** (detect new devices on bus)
- **Redundant controllers** (failover support)
- **Bridging** (multi-bus support via ESP32)

### 15.4 Features

- **Sleep/wake protocol** for power saving
- **Firmware update over bus** (bootloader integration)
- **Time synchronization** (controller broadcasts timestamp)
- **Bus sniffer/analyzer tool** (host application)
- **Wireshark dissector** for protocol analysis
- **Python bindings** for scripting and automation

### 15.5 Safety & Reliability

- **Watchdog integration** (detect bus hangs)
- **Error injection testing** (built-in self-test)
- **MISRA-C compliance** (for safety certification)
- **Functional safety** (ISO 26262, IEC 61508)

---

## 16. References

### 16.1 Related Documents

- `firmware/shared/tsumikoro_protocol.h` - Existing protocol definitions
- `firmware/tsumikoro-ministepper/` - STM32G071 motor controller
- `firmware/tsumikoro-servo/` - STM32G030 servo controller
- `firmware/tsumikoro-bridge/` - ESP32-S3 bridge firmware

### 16.2 Standards

- **RS-485**: TIA/EIA-485-A Standard (half-duplex serial)
- **Modbus RTU**: Similar use case for industrial control
- **CRC-8-CCITT**: ITU-T Recommendation (polynomial 0x07)
- **UART**: Universal Asynchronous Receiver/Transmitter

### 16.3 Datasheets

- STM32G030F6 Datasheet: https://www.st.com/resource/en/datasheet/stm32g030f6.pdf
- STM32G071G8 Datasheet: https://www.st.com/resource/en/datasheet/stm32g071g8.pdf
- ESP32-S3 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- MAX485 RS-485 Transceiver: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf

### 16.4 Software Libraries

- STM32 HAL UART: https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f0-hal-and-lowlayer-drivers-stmicroelectronics.pdf
- ESP-IDF UART Driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
- POSIX termios: https://man7.org/linux/man-pages/man3/termios.3.html

### 16.5 Testing Tools

- Unity Test Framework: http://www.throwtheswitch.org/unity
- CppUTest: https://cpputest.github.io/
- Wireshark: https://www.wireshark.org/
- PulseView (sigrok): https://sigrok.org/wiki/PulseView

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.5 | 2025-11-03 | Tsumikoro Project | Added section 4.7.8: Bus Scanning and Device Discovery with ping sweep, broadcast discovery, device information structure, performance analysis, smart scanning, and health monitoring strategies |
| 0.4 | 2025-11-03 | Tsumikoro Project | Updated protocol to use 16-bit command identifiers (0x0000-0xFFFF), removed product prefixes from command names, reorganized commands by functional class (Generic, Sensor, Stepper, Servo, DC Motor, Bridge), updated packet format (7-71 bytes), updated all API signatures |
| 0.3 | 2025-11-03 | Tsumikoro Project | Expanded section 4.4: Half-Duplex Control with TX echo, collision detection (CSMA/CD), random backoff, carrier sense, and exponential backoff strategies |
| 0.2 | 2025-11-03 | Tsumikoro Project | Added section 4.7: Command/Response Protocol with dispatch mechanism, ACK/NAK responses, timeout handling, and retry logic |
| 0.1 | 2025-11-03 | Tsumikoro Project | Initial draft |

---

**End of Document**
