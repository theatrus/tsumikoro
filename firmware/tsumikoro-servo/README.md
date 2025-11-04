# Tsumikoro Servo Controller

6-channel PWM servo controller with H-bridge motor driver for STM32G030F6P6.

## Features

### Servo Control
- **6 independent servo channels** using hardware PWM (TIM3, TIM14, TIM16, TIM17)
- **50Hz PWM** with 1-2ms pulse width range (standard servo timing)
- **0-180° position range** with 0.1° resolution (0-1800 units)
- Smooth position transitions with configurable speed
- Per-channel calibration (custom min/max pulse widths: 500-2500µs)
- Multi-servo synchronized movement command
- Zero CPU overhead (hardware PWM generation)

### H-Bridge Motor Driver
- **20kHz PWM** speed control using TIM1_CH4 (above audible range)
- **4 direction modes**: FORWARD, REVERSE, BRAKE, COAST
- **Speed range**: 0-1000 (0-100%)
- Emergency stop function with active braking
- Independent GPIO direction control

### Communication
- Multi-drop RS-485 bus protocol
- Hardware ID support (2-bit = 4 unique addresses)
- Tsumikoro protocol commands (0x3000-0x3FFF for servos, 0x4000-0x4FFF for motor)
- 1 Mbaud communication speed

## Hardware

### MCU
- **STM32G030F6P6TR** (TSSOP20)
  - ARM Cortex-M0+ @ 64MHz
  - 32KB Flash, 8KB RAM
  - Optimized with LL (Low-Level) drivers

### Pin Allocation

```
STM32G030F6P6 TSSOP20 Pinout
============================

                    ┌─────────────┐
  PB7 (HW ID 0)   ──│1   •    20│── PA14 (SWCLK)
  PB8 (HW ID 1)   ──│2        19│── PA13 (SWDIO)
  PB9 (Motor IN2) ──│3        18│── PA12 (Motor IN1)
 NRST             ──│4        17│── PA11 (Motor PWM)
 VDDA             ──│5        16│── PA8/PB0/PB1/PB2 (bonded)
  PA0 (Servo 0)   ──│6        15│── PA7 (Servo 5)
  PA1 (RS485 DE)  ──│7        14│── PA6 (Servo 4)
  PA2 (Servo 1)   ──│8        13│── PA5 (LED)
  GND             ──│9        12│── PA4 (Servo 3)
  VDD             ──│10       11│── PA3 (Servo 2)
                    └─────────────┘

Note: PA9/PA10 (USART TX/RX) are internal or alternate package pins
```

### Functional Block Diagram

```
SERVO OUTPUTS (50Hz PWM):          MOTOR CONTROL (20kHz PWM):
  ┌─────────────┐                    ┌──────────────┐
  │ Channel 0 ├─── PA0 (TIM3_CH1)    │ Speed PWM  ├─── PA11 (TIM1_CH4)
  │ Channel 1 ├─── PA2 (TIM3_CH3)    │ Direction 1├─── PA12 (IN1)
  │ Channel 2 ├─── PA3 (TIM3_CH4)    │ Direction 2├─── PB9  (IN2)
  │ Channel 3 ├─── PA4 (TIM14_CH1)   └──────────────┘
  │ Channel 4 ├─── PA6 (TIM16_CH1)
  │ Channel 5 ├─── PA7 (TIM17_CH1)   HARDWARE ID (2-bit):
  └─────────────┘                    ┌──────────────┐
                                     │ ID Bit 0   ├─── PB7 (input)
BUS COMMUNICATION:                   │ ID Bit 1   ├─── PB8 (input)
  ┌─────────────┐                    └──────────────┘
  │ TX        ├─── PA9  (USART1)
  │ RX        ├─── PA10 (USART1)     STATUS/DEBUG:
  │ DE (RS485)├─── PA1  (GPIO)       ┌──────────────┐
  └─────────────┘                    │ LED        ├─── PA5  (GPIO)
                                     │ SWDIO      ├─── PA13 (debug)
                                     │ SWCLK      ├─── PA14 (debug)
                                     └──────────────┘
```

### Complete Pin Mapping

| Physical Pin | GPIO | Function | Timer/AF | Direction |
|--------------|------|----------|----------|-----------|
| 1 | PB7 | Hardware ID Bit 0 | GPIO | Input |
| 2 | PB8 | Hardware ID Bit 1 | GPIO | Input |
| 3 | PB9 | Motor Direction IN2 | GPIO | Output |
| 4 | NRST | Reset | System | - |
| 5 | VDDA | Analog Power | Power | - |
| 6 | PA0 | Servo Channel 0 | TIM3_CH1 (AF1) | Output |
| 7 | PA1 | RS-485 DE | GPIO | Output |
| 8 | PA2 | Servo Channel 1 | TIM3_CH3 (AF1) | Output |
| 9 | GND | Ground | Power | - |
| 10 | VDD | Digital Power | Power | - |
| 11 | PA3 | Servo Channel 2 | TIM3_CH4 (AF1) | Output |
| 12 | PA4 | Servo Channel 3 | TIM14_CH1 (AF4) | Output |
| 13 | PA5 | Status LED | GPIO | Output |
| 14 | PA6 | Servo Channel 4 | TIM16_CH1 (AF5) | Output |
| 15 | PA7 | Servo Channel 5 | TIM17_CH1 (AF5) | Output |
| 16 | PA8/PB0/PB1/PB2 | **Reserved** (bonded) | - | Available |
| 17 | PA11 | Motor PWM Speed | TIM1_CH4 (AF2) | Output |
| 18 | PA12 | Motor Direction IN1 | GPIO | Output |
| 19 | PA13 | SWDIO (Debug) | SWD | Protected |
| 20 | PA14 | SWCLK (Debug) | SWD | Protected |
| Internal | PA9 | USART1 TX | USART1 | Output |
| Internal | PA10 | USART1 RX | USART1 | Input |

### Resource Utilization

**Memory:**
- Flash: 29,300 bytes / 32 KB (89.42%)
- RAM: 2,200 bytes / 8 KB (26.86%)

**Timers:**
- TIM1: H-bridge motor PWM (advanced timer)
- TIM3: Servo channels 0, 1, 2
- TIM14: Servo channel 3
- TIM16: Servo channel 4
- TIM17: Servo channel 5

**All timers allocated** - no expansion possible for more servo channels.

**Available:**
- Pin 16 (PA8/PB0/PB1/PB2 bonded group) - can be used for:
  - ADC input (current/voltage sensing)
  - MCO (clock output)
  - Additional TIM1 channels

## Building

From repository root:
```bash
# Using top-level Makefile
make build-servo

# Or manually with CMake
cd firmware
mkdir -p build/servo-g030
cd build/servo-g030
cmake ../.. -DPROJECT=tsumikoro-servo -DMCU=STM32G030
make -j$(nproc)
```

Output: `firmware/build/servo-g030/tsumikoro-servo.elf`

## Bus Protocol Commands

### Servo Commands (0x3000-0x3FFF)

**TSUMIKORO_CMD_SERVO_SET_POSITION (0x3001)**
- Set target position for a servo channel
- Data: `[channel_index, position_hi, position_lo]`
- Position: 0-1800 (0-180° in tenths)
- Response: `[0x00]` success, `[0xFF]` error

**TSUMIKORO_CMD_SERVO_GET_POSITION (0x3002)**
- Get current servo position
- Data: `[channel_index]`
- Response: `[position_hi, position_lo]`

**TSUMIKORO_CMD_SERVO_SET_SPEED (0x3003)**
- Set servo movement speed
- Data: `[channel_index, speed_hi, speed_lo]`
- Speed: tenths of degrees per process tick

**TSUMIKORO_CMD_SERVO_CALIBRATE (0x3005)**
- Set custom pulse width limits
- Data: `[channel_index, min_pulse_hi, min_pulse_lo, max_pulse_hi, max_pulse_lo]`
- Pulse widths: 500-2500µs

**TSUMIKORO_CMD_SERVO_ENABLE (0x3006)**
- Enable/disable servo output
- Data: `[channel_index, enable]`
- Enable: 0=disable, 1=enable

**TSUMIKORO_CMD_SERVO_SET_MULTI (0x3007)**
- Set multiple servos atomically
- Data: `[count, ch0, pos0_hi, pos0_lo, ch1, pos1_hi, pos1_lo, ...]`

**TSUMIKORO_CMD_SERVO_GET_STATUS (0x3008)**
- Get detailed channel status
- Data: `[channel_index]`
- Response: `[enabled, moving, current_pos_hi, current_pos_lo, target_pos_hi, target_pos_lo]`

### DC Motor Commands (0x4000-0x4FFF)

**TSUMIKORO_CMD_DCMOTOR_SET (0x4001)**
- Set motor speed and direction
- Data: `[speed_hi, speed_lo, direction]`
- Speed: 0-1000 (0-100%)
- Direction: 0=FORWARD, 1=REVERSE, 2=BRAKE, 3=COAST

**TSUMIKORO_CMD_DCMOTOR_GET_SPEED (0x4002)**
- Get current motor speed
- Response: `[speed_hi, speed_lo]`

**TSUMIKORO_CMD_DCMOTOR_GET_DIRECTION (0x4003)**
- Get current motor direction
- Response: `[direction]`

**TSUMIKORO_CMD_DCMOTOR_ENABLE (0x4004)**
- Enable/disable motor output
- Data: `[enable]`
- Enable: 0=disable (coast), 1=enable

**TSUMIKORO_CMD_DCMOTOR_ESTOP (0x4005)**
- Emergency stop (immediate brake)
- No data required

## Programming

### Using SWD
Connect ST-Link or compatible programmer to PA13 (SWDIO) and PA14 (SWCLK).

```bash
# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
        -c "program build/servo-g030/tsumikoro-servo.elf verify reset exit"
```

### Using STM32CubeProgrammer
Load `tsumikoro-servo.elf` or `tsumikoro-servo.hex` and flash via SWD.

## Hardware ID Configuration

The 2-bit hardware ID (PB7/PB8) allows 4 unique device addresses:

| PB8 | PB7 | ID | Use Case |
|-----|-----|-----|----------|
| 0 | 0 | 0x02 | Primary servo controller |
| 0 | 1 | 0x03 | Secondary servo controller |
| 1 | 0 | 0x04 | Tertiary servo controller |
| 1 | 1 | 0x05 | Quaternary servo controller |

Connect PB7/PB8 to GND or VDD with pull resistors to set ID.

## Technical Details

### PWM Timing

**Servos (50Hz):**
- Period: 20ms (20,000µs)
- Timer clock: 1 MHz (64 MHz / 64 prescaler)
- Pulse width: 1000-2000µs (default), configurable 500-2500µs
- Resolution: 1µs steps

**Motor (20kHz):**
- Period: 50µs
- Timer clock: 1 MHz (64 MHz / 64 prescaler)
- Pulse width: 0-50µs (0-100% duty cycle)
- Resolution: 1µs steps (2% duty cycle per step)

### Direction Control Truth Table

| IN1 | IN2 | Result |
|-----|-----|--------|
| H | L | Forward |
| L | H | Reverse |
| H | H | Brake |
| L | L | Coast |

## License

Copyright (c) 2025 Yann Ramin
Licensed under Apache 2.0
