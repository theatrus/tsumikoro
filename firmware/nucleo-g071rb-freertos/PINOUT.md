# NUCLEO-G071RB Pinout Guide

This document describes all external connections for the NUCLEO-G071RB development board firmware with Tsumikoro bus protocol and TMC2130 stepper driver support.

## Hardware Requirements

- **NUCLEO-G071RB** development board (STM32G071RBT6)
- **RS-485 transceiver** module (e.g., MAX485, SN65HVD72) for bus communication
- **TMC2130** stepper driver module (SPI interface)
- **Stepper motor** (compatible with TMC2130)
- **Power supply** (12-24V for motor, 5V for logic)

## Board Overview

The NUCLEO-G071RB has three main connector sections:
- **CN7/CN10**: Arduino Uno R3 compatible headers (left side)
- **CN8/CN9**: Morpho headers with additional pins (both sides)
- **CN1**: ST-Link USB connector (top)

## Complete Pin Mapping

### On-Board Peripherals

| Function | Pin | Arduino | Morpho | Notes |
|----------|-----|---------|--------|-------|
| User LED (LD4) | PA5 | D13 | CN10-11 | Green LED, active high |
| User Button (B1) | PC13 | - | CN7-23 | Blue button, active high |
| ST-Link UART TX | PA2 | - | CN10-35 | Debug output (115200 baud) |
| ST-Link UART RX | PA3 | - | CN10-37 | Debug input (115200 baud) |

### Tsumikoro Bus Protocol (UART1 + RS-485)

| Function | Pin | Arduino | Morpho | Signal |
|----------|-----|---------|--------|--------|
| Bus TX | PC4 | D1 | CN9-34 | UART1_TX → RS-485 DI |
| Bus RX | PC5 | D0 | CN9-6 | UART1_RX ← RS-485 RO |
| Driver Enable | PA1 | - | CN8-32 | RS-485 DE/RE control |

**Bus Configuration:**
- Baud rate: 1 Mbaud
- Device ID: 0x10 (configurable in firmware)
- Half-duplex RS-485 (2-wire bus)

### TMC2130 Stepper Driver (SPI1)

| Function | Pin | Arduino | Morpho | Signal |
|----------|-----|---------|--------|--------|
| SPI SCK | PB3 | D3 | CN10-31 | SPI clock |
| SPI MISO | PB4 | D5 | CN10-27 | SPI data in |
| SPI MOSI | PB5 | D4 | CN10-29 | SPI data out |
| Chip Select | PB0 | - | CN9-34 | Active low |
| Step Pulse | PB1 | - | CN9-18 | Step signal |
| Direction | PB2 | D10 | CN9-22 | Direction control |
| Enable | PA0 | A0 | CN8-28 | Active low (inverted) |

**SPI Configuration:**
- Speed: 1 MHz
- Mode: 3 (CPOL=1, CPHA=1)
- 8-bit transfers

### Power Connections

| Function | Pin | Arduino | Morpho | Voltage |
|----------|-----|---------|--------|---------|
| +5V | - | +5V | CN7-18 | Logic power |
| +3.3V | - | +3.3V | CN7-16 | Available (not used) |
| GND | - | GND | CN7-20 | Ground reference |

## Wiring Diagrams

### RS-485 Transceiver Connection

```
NUCLEO-G071RB              MAX485 Module
--------------              -------------
PC4 (TX) -----------------> DI (Driver Input)
PC5 (RX) <----------------- RO (Receiver Output)
PA1 (DE) ------+----------> DE (Driver Enable)
               +----------> RE (Receiver Enable, inverted)
+5V ------------------------> VCC
GND ------------------------> GND

MAX485 Screw Terminals
----------------------
A    ---[=== Bus Line A (differential pair)
B    ---[=== Bus Line B (differential pair)
```

**Notes:**
- Connect DE and RE together on MAX485 (half-duplex mode)
- Use twisted pair cable for A/B bus lines
- Add 120Ω termination resistor at both ends of long bus runs
- Keep bus wiring away from motor power cables

### TMC2130 Stepper Driver Connection

```
NUCLEO-G071RB              TMC2130 Module            Stepper Motor
--------------              --------------            -------------
PB3 (SCK) ----------------> SCK
PB4 (MISO) <--------------- SDO
PB5 (MOSI) ---------------> SDI
PB0 (CS) -----------------> CSN
PB1 (STEP) ---------------> STEP
PB2 (DIR) ----------------> DIR
PA0 (EN) -----------------> EN
+5V -----------------------> VIO (logic)
GND -----------------------> GND -------------------- Motor GND
                            VM (12-24V) ------------- Motor Supply +
                            GND -------------------- Motor Supply -

                            1A --------------------> Coil A+
                            1B --------------------> Coil A-
                            2A --------------------> Coil B+
                            2B --------------------> Coil B-
```

**TMC2130 Configuration (via SPI):**
- Microsteps: 256 with interpolation
- StealthChop: Enabled (quiet operation)
- Run current (IRUN): 31/32 = ~97%
- Hold current (IHOLD): 16/32 = ~50%

**Motor Power:**
- VM voltage: 12-24V DC (depends on motor rating)
- Current limit: Set by firmware (adjust IRUN/IHOLD)
- Use electrolytic capacitor (100μF+) near TMC2130 VM pin

### Debug UART (ST-Link VCP)

The debug UART is automatically connected via the ST-Link USB interface:
- No external wiring needed
- Connect USB cable from CN1 to PC
- Serial port appears as `/dev/ttyACM0` (Linux) or `COMx` (Windows)
- Settings: 115200 baud, 8N1

## Pin Allocation Summary

### Port A Pins
```
PA0  : TMC_EN (Enable, active low)
PA1  : BUS_DE (RS-485 Driver Enable)
PA2  : USART2_TX (ST-Link debug)
PA3  : USART2_RX (ST-Link debug)
PA4  : Available
PA5  : LED (LD4, on-board)
PA6  : Available
PA7  : Available
PA8  : Available
PA9  : Available
PA10 : Available
PA11 : Available (SWDIO reserved for debug)
PA12 : Available
PA13 : Available (SWCLK reserved for debug)
PA14 : Available
PA15 : Available
```

### Port B Pins
```
PB0  : TMC_CS (SPI Chip Select)
PB1  : TMC_STEP (Step pulse)
PB2  : TMC_DIR (Direction)
PB3  : SPI1_SCK
PB4  : SPI1_MISO
PB5  : SPI1_MOSI
PB6  : Available
PB7  : Available
PB8  : Available
PB9  : Available
PB10 : Available
PB11 : Available
PB12 : Available
PB13 : Available
PB14 : Available
PB15 : Available
```

### Port C Pins
```
PC4  : USART1_TX (Bus TX)
PC5  : USART1_RX (Bus RX)
PC6  : Available
PC7  : Available
PC13 : Button (B1, on-board)
PC14 : OSC32_IN (reserved)
PC15 : OSC32_OUT (reserved)
```

## Available GPIO Pins

The following pins are available for expansion:
- **Port A**: PA4, PA6, PA7, PA8, PA9, PA10, PA12, PA14, PA15
- **Port B**: PB6-PB15
- **Port C**: PC6, PC7

## Example Wiring Setup

### Minimal Development Setup

1. **NUCLEO board**: Connect USB cable to CN1 (ST-Link)
2. **RS-485 transceiver**:
   - Use breadboard or breakout module
   - Connect as shown in wiring diagram above
   - Connect to bus via screw terminals
3. **TMC2130 driver**:
   - Mount on breadboard or use module
   - Connect SPI and control signals
   - Connect motor power supply (12-24V)
   - Connect stepper motor
4. **Power**:
   - USB powers NUCLEO (5V logic)
   - Separate 12-24V supply for motor via TMC2130

### Production PCB Recommendations

- Place RS-485 transceiver close to NUCLEO TX/RX pins
- Use differential routing for RS-485 A/B signals
- Add TVS diodes on RS-485 bus lines for ESD protection
- Separate motor power GND and logic GND, connect at single point
- Add bulk capacitance (100-470μF) on motor power supply
- Route SPI signals with matched lengths (not critical at 1 MHz)
- Add pull-up resistors on I2C lines if using expansion

## Electrical Specifications

### Logic Levels
- **GPIO High**: 2.4V min (3.3V nominal)
- **GPIO Low**: 0.4V max
- **Input tolerance**: 5V tolerant on most pins (check datasheet)

### Current Limits
- **Per GPIO**: 20 mA max
- **Total GPIO**: 120 mA max across all ports
- **3.3V regulator**: 500 mA max from ST-Link
- **5V supply**: 1000 mA max total (USB limited)

### Motor Power
- **TMC2130 VM**: 12-24V (check motor and driver ratings)
- **Peak current**: Set by firmware IRUN parameter
- **Use separate motor power supply** - do not power from NUCLEO

## Safety Warnings

⚠️ **IMPORTANT SAFETY NOTES:**

1. **Motor Power Isolation**
   - NEVER connect motor power supply to NUCLEO 5V/3.3V
   - Use separate isolated power supply for motor
   - Connect GND between supplies (star ground recommended)

2. **Voltage Levels**
   - TMC2130 VIO must be 3.3V or 5V (use 5V from NUCLEO)
   - RS-485 transceivers typically 5V logic
   - Verify logic levels before connecting

3. **Current Protection**
   - Do not exceed GPIO current limits
   - Use current-limiting resistors for LEDs
   - Fuses recommended on motor power supply

4. **ESD Protection**
   - Handle TMC2130 with ESD precautions
   - Add TVS diodes on external connections
   - Use proper grounding

## Testing Procedure

### 1. Power-On Test (No Motor)
```bash
# Connect ST-Link USB, open serial terminal
screen /dev/ttyACM0 115200

# Expected output:
# ========================================
# NUCLEO-G071RB Tsumikoro DevBoard
# STM32G071RBT6 @ 64 MHz
# TMC2130 Stepper: SPI1 + Motion Control
# ========================================
```

### 2. SPI Communication Test
```python
# Send GET_VERSION command via bus
# Device should respond with firmware version
```

### 3. Motor Test (With Motor Connected)
```python
# Send STEPPER_ENABLE command
# Send STEPPER_MOVE command with small position
# Motor should move smoothly
```

## Troubleshooting

### No Debug Output
- Check USB cable connection
- Verify ST-Link drivers installed
- Try different serial port settings
- Check PA2/PA3 not shorted

### Bus Communication Fails
- Verify RS-485 wiring polarity (A/B)
- Check termination resistors (120Ω at ends)
- Verify baud rate matches (1 Mbaud)
- Check DE pin toggles with oscilloscope

### Stepper Motor Not Moving
- Verify TMC2130 power (VM and VIO LEDs if present)
- Check SPI communication (oscilloscope on SCK/MOSI)
- Verify motor wiring (measure coil resistance)
- Check EN pin is low (motor enabled)
- Increase IRUN current setting

### Stepper Motor Vibrates/Skips
- Reduce speed/acceleration parameters
- Increase motor current (IRUN)
- Check motor power supply voltage
- Verify mechanical load not too high

## References

- [NUCLEO-G071RB User Manual (UM2324)](https://www.st.com/resource/en/user_manual/um2324-stm32-nucleo64-boards-mb1360-stmicroelectronics.pdf)
- [STM32G071RB Datasheet](https://www.st.com/resource/en/datasheet/stm32g071rb.pdf)
- [TMC2130 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2130_datasheet_rev1.15.pdf)
- [MAX485 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf)

## Revision History

- **2025-01-08**: Initial version with TMC2130 stepper driver support
