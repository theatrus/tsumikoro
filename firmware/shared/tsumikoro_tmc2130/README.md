# TMC2130 Stepper Motor Driver Library

Platform-independent C library for the Trinamic TMC2130 stepper motor driver IC.

## Features

- **SPI Communication**: Full register access via 40-bit SPI datagrams
- **Step/Dir Control**: Direct step and direction pin control
- **Current Control**: Configurable hold and run currents (0-31 scale)
- **Microstepping**: Support for 256, 128, 64, 32, 16, 8, 4, 2, and full step modes
- **StealthChop**: Silent operation mode with voltage PWM
- **SpreadCycle**: Classic chopper mode for high dynamics
- **StallGuard**: Sensorless load detection for homing
- **Diagnostics**: Overtemperature, short circuit, open load detection
- **Platform Independent**: Uses callbacks for hardware abstraction

## Hardware Interface

### TMC2130 Pins

| Pin | Function | Description |
|-----|----------|-------------|
| CLK | SPI Clock | SPI clock input |
| SDI | SPI MOSI | SPI data input |
| SDO | SPI MISO | SPI data output |
| CS | Chip Select | Active low SPI chip select |
| STEP | Step Input | Step pulse input (active on rising edge) |
| DIR | Direction | Motor direction control |
| EN | Enable | Driver enable (active low) |

### Typical Wiring

```
MCU          TMC2130
----         -------
SPI_SCK  --> CLK
SPI_MOSI --> SDI
SPI_MISO <-- SDO
GPIO_CS  --> CS
GPIO_STEP --> STEP
GPIO_DIR  --> DIR
GPIO_EN   --> EN
```

## Usage

### 1. Include the Header

```c
#include "tsumikoro_tmc2130.h"
```

### 2. Implement Platform Callbacks

The library requires platform-specific callbacks for SPI and GPIO:

```c
/* Example for STM32 HAL */

static SPI_HandleTypeDef hspi1;

void my_spi_begin(void) {
    HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_RESET);
}

void my_spi_end(void) {
    HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_SET);
}

uint8_t my_spi_transfer(uint8_t data) {
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100);
    return rx_data;
}

void my_set_step_pin(bool state) {
    HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin,
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void my_set_dir_pin(bool state) {
    HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin,
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void my_set_enable_pin(bool state) {
    /* Note: EN pin is active low, but library handles inversion */
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin,
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void my_delay_us(uint32_t us) {
    /* Platform-specific microsecond delay */
    for (volatile uint32_t i = 0; i < us * 10; i++);
}
```

### 3. Initialize the Driver

```c
tsumikoro_tmc2130_t tmc = {0};

/* Assign callbacks */
tmc.spi_begin = my_spi_begin;
tmc.spi_end = my_spi_end;
tmc.spi_transfer = my_spi_transfer;
tmc.set_step_pin = my_set_step_pin;
tmc.set_dir_pin = my_set_dir_pin;
tmc.set_enable_pin = my_set_enable_pin;
tmc.delay_us = my_delay_us;

/* Initialize with default settings */
if (!tsumikoro_tmc2130_init(&tmc)) {
    /* Handle error */
}
```

### 4. Configure the Driver

```c
/* Set motor currents (0-31 scale)
 * IHOLD = 10 (10/32 of max current when motor stopped)
 * IRUN = 20 (20/32 of max current when motor running)
 * IHOLDDELAY = 2 (power down delay) */
tsumikoro_tmc2130_set_current(&tmc, 10, 20, 2);

/* Set 16 microsteps with 256 interpolation */
tsumikoro_tmc2130_set_microsteps(&tmc, TMC2130_MRES_16, true);

/* Enable stealthChop for silent operation */
tsumikoro_tmc2130_set_stealth_chop(&tmc, true);

/* Enable the driver */
tsumikoro_tmc2130_enable(&tmc, true);
```

### 5. Move the Motor

```c
/* Set direction */
tsumikoro_tmc2130_set_direction(&tmc, TMC2130_DIR_FORWARD);

/* Generate steps */
for (int i = 0; i < 200; i++) {  /* 200 steps = 1 revolution */
    tsumikoro_tmc2130_step(&tmc);
    /* Add delay between steps for desired speed */
    my_delay_us(1000);  /* 1ms = 1000 steps/sec */
}
```

### 6. Read Diagnostics

```c
/* Check for errors */
bool has_error = false;
if (tsumikoro_tmc2130_has_error(&tmc, &has_error)) {
    if (has_error) {
        /* Read full status */
        uint32_t status = 0;
        tsumikoro_tmc2130_read_status(&tmc, &status);

        if (status & TMC2130_DRV_STATUS_OT) {
            /* Overtemperature */
        }
        if (status & TMC2130_DRV_STATUS_S2GA) {
            /* Short to ground on coil A */
        }
        /* ... check other error bits */
    }
}

/* Read StallGuard value for sensorless homing */
uint16_t sg_result = 0;
if (tsumikoro_tmc2130_get_stallguard(&tmc, &sg_result)) {
    if (sg_result < 10) {
        /* Motor stalled or high load */
    }
}

/* Check if motor is at standstill */
bool standstill = false;
if (tsumikoro_tmc2130_is_standstill(&tmc, &standstill)) {
    if (standstill) {
        /* Motor stopped moving */
    }
}
```

## API Reference

### Initialization

#### `tsumikoro_tmc2130_init()`
Initialize TMC2130 with default settings.

**Parameters:**
- `tmc`: Pointer to TMC2130 handle (must have callbacks assigned)

**Returns:** `true` if successful, `false` on error

**Default Settings:**
- IRUN = 31 (maximum current)
- IHOLD = 0 (minimal standstill current)
- TOFF = 8 (chopper off time)
- 256 microsteps
- SpreadCycle mode

### Register Access

#### `tsumikoro_tmc2130_read_register()`
Read a TMC2130 register via SPI.

**Note:** TMC2130 returns previous register value, requires 2 transactions.

#### `tsumikoro_tmc2130_write_register()`
Write a TMC2130 register via SPI.

### Motion Control

#### `tsumikoro_tmc2130_set_direction()`
Set motor direction (FORWARD or REVERSE).

#### `tsumikoro_tmc2130_step()`
Generate a single step pulse.

**Note:** Step pulse duration is ~2µs high, ~2µs low.

#### `tsumikoro_tmc2130_enable()`
Enable or disable motor driver.

**Parameters:**
- `enable`: `true` to enable driver, `false` to disable

**Note:** EN pin is active low (handled internally).

### Configuration

#### `tsumikoro_tmc2130_set_current()`
Set motor hold and run currents.

**Parameters:**
- `ihold`: Hold current (0-31, where 31 = 32/32 of max)
- `irun`: Run current (0-31, where 31 = 32/32 of max)
- `iholddelay`: Power down delay (0-15)

**Current Calculation:**
```
I_RMS = (CS + 1) / 32 × V_FS / (R_SENSE + 0.02Ω) × 1/√2
```

Where:
- CS = current scale (ihold or irun)
- V_FS = full scale voltage (0.325V or 0.180V, set by VSENSE bit)
- R_SENSE = external sense resistor value

#### `tsumikoro_tmc2130_set_microsteps()`
Set microstep resolution.

**Parameters:**
- `mres`: Microstep resolution (TMC2130_MRES_xxx)
- `interpolate`: Enable 256 microstep interpolation

**Available Resolutions:**
- `TMC2130_MRES_256`: 256 microsteps
- `TMC2130_MRES_128`: 128 microsteps
- `TMC2130_MRES_64`: 64 microsteps
- `TMC2130_MRES_32`: 32 microsteps
- `TMC2130_MRES_16`: 16 microsteps (common)
- `TMC2130_MRES_8`: 8 microsteps
- `TMC2130_MRES_4`: 4 microsteps
- `TMC2130_MRES_2`: 2 microsteps (half step)
- `TMC2130_MRES_FULLSTEP`: Full step

#### `tsumikoro_tmc2130_set_stealth_chop()`
Enable or disable stealthChop mode.

**Parameters:**
- `enable`: `true` for stealthChop (silent), `false` for SpreadCycle (dynamic)

**StealthChop vs SpreadCycle:**
- **StealthChop**: Silent operation using voltage PWM, ideal for low speeds
- **SpreadCycle**: Higher dynamics and efficiency, audible chopping

### Diagnostics

#### `tsumikoro_tmc2130_read_status()`
Read DRV_STATUS register (0x6F).

#### `tsumikoro_tmc2130_is_standstill()`
Check if motor is in standstill.

#### `tsumikoro_tmc2130_has_error()`
Check for driver errors.

**Detects:**
- Overtemperature (OT)
- Short to ground (S2GA, S2GB)
- Open load (OLA, OLB)

#### `tsumikoro_tmc2130_get_stallguard()`
Get StallGuard result value (0-1023).

**Usage:** Higher values = lower motor load. Use for sensorless homing.

## Register Map

### Key Registers

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | GCONF | Global configuration |
| 0x01 | GSTAT | Global status flags |
| 0x04 | IOIN | Input pin status |
| 0x10 | IHOLD_IRUN | Driver current control |
| 0x6C | CHOPCONF | Chopper configuration |
| 0x6D | COOLCONF | CoolStep configuration |
| 0x6F | DRV_STATUS | Driver status flags |
| 0x70 | PWMCONF | StealthChop PWM config |

### GCONF Bits (0x00)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | I_SCALE_ANALOG | Use external VREF |
| 2 | EN_PWM_MODE | Enable stealthChop |
| 4 | SHAFT | Inverse motor direction |
| 7 | DIAG0_STALL | DIAG0 outputs stallGuard |
| 8 | DIAG1_STALL | DIAG1 outputs stallGuard |

### DRV_STATUS Bits (0x6F)

| Bit | Name | Description |
|-----|------|-------------|
| [9:0] | SG_RESULT | StallGuard result |
| 24 | STALLGUARD | StallGuard status |
| 25 | OT | Overtemperature flag |
| 26 | OTPW | Overtemperature pre-warning |
| 27 | S2GA | Short to ground coil A |
| 28 | S2GB | Short to ground coil B |
| 29 | OLA | Open load coil A |
| 30 | OLB | Open load coil B |
| 31 | STST | Standstill detected |

## Example: Sensorless Homing

```c
/* Configure StallGuard threshold in COOLCONF register */
uint32_t coolconf = 0;
tsumikoro_tmc2130_read_register(&tmc, TMC2130_REG_COOLCONF, &coolconf);
coolconf |= (20 << 16);  /* Set SGT (StallGuard threshold) */
tsumikoro_tmc2130_write_register(&tmc, TMC2130_REG_COOLCONF, coolconf);

/* Move until stall detected */
tsumikoro_tmc2130_set_direction(&tmc, TMC2130_DIR_REVERSE);
tsumikoro_tmc2130_enable(&tmc, true);

while (1) {
    tsumikoro_tmc2130_step(&tmc);
    my_delay_us(500);  /* Slow speed for homing */

    uint16_t sg_result = 0;
    if (tsumikoro_tmc2130_get_stallguard(&tmc, &sg_result)) {
        if (sg_result < 10) {
            /* Stall detected - home position found */
            break;
        }
    }
}

/* Back off a few steps */
tsumikoro_tmc2130_set_direction(&tmc, TMC2130_DIR_FORWARD);
for (int i = 0; i < 50; i++) {
    tsumikoro_tmc2130_step(&tmc);
    my_delay_us(1000);
}
```

## Integration with CMake

In your project's `CMakeLists.txt`:

```cmake
# Add TMC2130 library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_tmc2130
                 ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_tmc2130)

# Link to your executable
target_link_libraries(${PROJECT_NAME}.elf
    tsumikoro_tmc2130
)
```

## References

- [TMC2130 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2130_datasheet_rev1.15.pdf)
- [Trinamic Motion Control](https://www.analog.com/en/products/tmc2130.html)
- [StallGuard Application Note](https://www.trinamic.com/support/eval-kits/details/tmc2130-eval/)

## License

Part of the Tsumikoro project.
