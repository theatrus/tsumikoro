# Trinamic TMC2130/TMC5160 Stepper Motor Driver Library

Platform-independent C library for Trinamic stepper motor drivers supporting both TMC2130 (Step/Dir) and TMC5160 (integrated motion controller).

## Supported Chips

### TMC2130
- **Type**: Stepper motor driver with SPI configuration
- **Control**: External Step/Dir signals
- **Current**: Up to ~1.4A RMS per coil
- **Features**: StealthChop, SpreadCycle, StallGuard, CoolStep

### TMC5160
- **Type**: Stepper motor driver with integrated motion controller
- **Control**: SPI position/velocity commands OR Step/Dir
- **Current**: Several amps with external MOSFETs
- **Features**: All TMC2130 features PLUS:
  - 6-point ramp generator (autonomous motion)
  - Position and velocity registers
  - Automatic acceleration/deceleration
  - Advanced motion profiling

## Common Features

- **SPI Communication**: Full register access via 40-bit SPI datagrams
- **Step/Dir Control**: Direct step and direction pin control (both chips)
- **Current Control**: Configurable hold and run currents (0-31 scale)
- **Microstepping**: Support for 256, 128, 64, 32, 16, 8, 4, 2, and full step modes
- **StealthChop**: Silent operation mode with voltage PWM
- **SpreadCycle**: Classic chopper mode for high dynamics
- **StallGuard**: Sensorless load detection for homing
- **Diagnostics**: Overtemperature, short circuit, open load detection
- **Platform Independent**: Uses callbacks for hardware abstraction

## Hardware Interface

### Pin Configuration (Both Chips)

| Pin | Function | Description |
|-----|----------|-------------|
| CLK | SPI Clock | SPI clock input |
| SDI | SPI MOSI | SPI data input |
| SDO | SPI MISO | SPI data output |
| CS | Chip Select | Active low SPI chip select |
| STEP | Step Input | Step pulse input (TMC2130 required, TMC5160 optional) |
| DIR | Direction | Motor direction control (optional with TMC5160 motion controller) |
| EN | Enable | Driver enable (active low) |

### Typical Wiring

```
MCU          TMC2130/TMC5160
----         ---------------
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
#include "tsumikoro_tmc_stepper.h"
```

### 2. Implement Platform Callbacks

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
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin,
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void my_delay_us(uint32_t us) {
    /* Platform-specific microsecond delay */
    for (volatile uint32_t i = 0; i < us * 10; i++);
}
```

### 3. Initialize the Driver

#### TMC2130 (Step/Dir Mode)

```c
tsumikoro_tmc_stepper_t tmc = {0};

/* Assign callbacks */
tmc.spi_begin = my_spi_begin;
tmc.spi_end = my_spi_end;
tmc.spi_transfer = my_spi_transfer;
tmc.set_step_pin = my_set_step_pin;
tmc.set_dir_pin = my_set_dir_pin;
tmc.set_enable_pin = my_set_enable_pin;
tmc.delay_us = my_delay_us;

/* Initialize as TMC2130 */
if (!tsumikoro_tmc_init(&tmc, TMC_CHIP_TMC2130)) {
    /* Handle error */
}
```

#### TMC5160 (Motion Controller Mode)

```c
tsumikoro_tmc_stepper_t tmc = {0};

/* Assign callbacks (same as TMC2130) */
tmc.spi_begin = my_spi_begin;
tmc.spi_end = my_spi_end;
tmc.spi_transfer = my_spi_transfer;
tmc.set_enable_pin = my_set_enable_pin;
tmc.delay_us = my_delay_us;

/* Note: STEP and DIR callbacks optional for TMC5160 motion controller */

/* Initialize as TMC5160 */
if (!tsumikoro_tmc_init(&tmc, TMC_CHIP_TMC5160)) {
    /* Handle error */
}
```

### 4. Configure the Driver (Common API)

```c
/* Set motor currents (0-31 scale) */
tsumikoro_tmc_set_current(&tmc, 10, 20, 2);

/* Set 16 microsteps with 256 interpolation */
tsumikoro_tmc_set_microsteps(&tmc, TMC_MRES_16, true);

/* Enable stealthChop for silent operation */
tsumikoro_tmc_set_stealth_chop(&tmc, true);

/* Enable the driver */
tsumikoro_tmc_enable(&tmc, true);
```

### 5A. TMC2130: Move Motor with Step/Dir

```c
/* Set direction */
tsumikoro_tmc_set_direction(&tmc, TMC_DIR_FORWARD);

/* Generate steps */
for (int i = 0; i < 200; i++) {  /* 200 full steps = 1 revolution */
    tsumikoro_tmc_step(&tmc);
    my_delay_us(1000);  /* 1ms = 1000 steps/sec */
}
```

### 5B. TMC5160: Move Motor with Motion Controller

```c
/* Configure ramp parameters (velocities and accelerations) */
tsumikoro_tmc5160_set_ramp_params(&tmc,
    1,      /* VSTART: Starting velocity */
    1000,   /* A1: First acceleration */
    30000,  /* V1: First acceleration/deceleration threshold */
    1000,   /* AMAX: Maximum acceleration */
    50000,  /* VMAX: Maximum velocity */
    1000,   /* DMAX: Maximum deceleration */
    1000,   /* D1: First deceleration */
    10      /* VSTOP: Stop velocity */
);

/* Move to absolute position */
tsumikoro_tmc5160_move_to(&tmc, 3200);  /* 3200 microsteps */

/* Wait for motion to complete */
bool reached = false;
while (!reached) {
    tsumikoro_tmc5160_position_reached(&tmc, &reached);
    my_delay_us(1000);
}

/* Get current position and velocity */
int32_t pos, vel;
tsumikoro_tmc5160_get_position(&tmc, &pos);
tsumikoro_tmc5160_get_velocity(&tmc, &vel);
```

### 6. Read Diagnostics (Common API)

```c
/* Check for errors */
bool has_error = false;
if (tsumikoro_tmc_has_error(&tmc, &has_error)) {
    if (has_error) {
        uint32_t status = 0;
        tsumikoro_tmc_read_status(&tmc, &status);

        if (status & TMC_DRV_STATUS_OT) {
            /* Overtemperature */
        }
        if (status & TMC_DRV_STATUS_S2GA) {
            /* Short to ground on coil A */
        }
    }
}

/* Read StallGuard for sensorless homing */
uint16_t sg_result = 0;
if (tsumikoro_tmc_get_stallguard(&tmc, &sg_result)) {
    if (sg_result < 10) {
        /* Motor stalled or high load */
    }
}
```

## API Reference

### Common Functions (Both Chips)

| Function | Description |
|----------|-------------|
| `tsumikoro_tmc_init()` | Initialize driver with chip type |
| `tsumikoro_tmc_read_register()` | Read SPI register |
| `tsumikoro_tmc_write_register()` | Write SPI register |
| `tsumikoro_tmc_set_direction()` | Set motor direction |
| `tsumikoro_tmc_step()` | Generate single step pulse |
| `tsumikoro_tmc_enable()` | Enable/disable driver |
| `tsumikoro_tmc_set_current()` | Set hold/run currents |
| `tsumikoro_tmc_set_microsteps()` | Set microstep resolution |
| `tsumikoro_tmc_set_stealth_chop()` | Enable/disable stealthChop |
| `tsumikoro_tmc_read_status()` | Read DRV_STATUS register |
| `tsumikoro_tmc_is_standstill()` | Check standstill state |
| `tsumikoro_tmc_has_error()` | Check for errors |
| `tsumikoro_tmc_get_stallguard()` | Get StallGuard value |

### TMC5160-Specific Functions (Motion Controller)

| Function | Description |
|----------|-------------|
| `tsumikoro_tmc5160_set_ramp_mode()` | Set ramp mode (positioning/velocity) |
| `tsumikoro_tmc5160_set_ramp_params()` | Configure ramp parameters |
| `tsumikoro_tmc5160_move_to()` | Move to absolute position |
| `tsumikoro_tmc5160_get_position()` | Read current position |
| `tsumikoro_tmc5160_set_position()` | Set current position (no movement) |
| `tsumikoro_tmc5160_get_velocity()` | Read current velocity |
| `tsumikoro_tmc5160_position_reached()` | Check if target reached |
| `tsumikoro_tmc5160_stop()` | Stop motor immediately |

### Legacy TMC2130 Compatibility

For backward compatibility, all original TMC2130 functions are available as inline wrappers:

```c
/* These work identically to the new API */
tsumikoro_tmc2130_init(&tmc);  /* Calls tsumikoro_tmc_init(tmc, TMC_CHIP_TMC2130) */
tsumikoro_tmc2130_set_current(&tmc, 10, 20, 2);
tsumikoro_tmc2130_step(&tmc);
/* etc. */
```

## Register Map

### Shared Registers (TMC2130 and TMC5160)

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | GCONF | Global configuration |
| 0x01 | GSTAT | Global status flags |
| 0x10 | IHOLD_IRUN | Driver current control |
| 0x6C | CHOPCONF | Chopper configuration |
| 0x6D | COOLCONF | CoolStep configuration |
| 0x6F | DRV_STATUS | Driver status flags |
| 0x70 | PWMCONF | StealthChop PWM config |

### TMC5160 Motion Control Registers

| Address | Name | Description |
|---------|------|-------------|
| 0x20 | RAMPMODE | Ramp generator mode |
| 0x21 | XACTUAL | Current position |
| 0x22 | VACTUAL | Current velocity |
| 0x23 | VSTART | Starting velocity |
| 0x26 | AMAX | Maximum acceleration |
| 0x27 | VMAX | Maximum velocity |
| 0x28 | DMAX | Maximum deceleration |
| 0x2B | VSTOP | Stop velocity |
| 0x2D | XTARGET | Target position |
| 0x35 | RAMP_STAT | Ramp status flags |

## Example: TMC5160 Autonomous Motion

```c
/* Initialize TMC5160 */
tsumikoro_tmc_stepper_t tmc = {0};
/* ... assign callbacks ... */
tsumikoro_tmc_init(&tmc, TMC_CHIP_TMC5160);

/* Configure for smooth motion */
tsumikoro_tmc_set_current(&tmc, 5, 15, 2);
tsumikoro_tmc_set_microsteps(&tmc, TMC_MRES_256, true);  /* 256 µsteps */
tsumikoro_tmc_set_stealth_chop(&tmc, true);

/* Set ramp parameters */
tsumikoro_tmc5160_set_ramp_params(&tmc,
    10,     /* VSTART */
    2000,   /* A1 */
    50000,  /* V1 */
    2000,   /* AMAX */
    100000, /* VMAX */
    2000,   /* DMAX */
    2000,   /* D1 */
    10      /* VSTOP */
);

/* Enable driver */
tsumikoro_tmc_enable(&tmc, true);

/* Move to position 51200 (10 revolutions @ 256 µsteps) */
tsumikoro_tmc5160_move_to(&tmc, 51200);

/* Motion happens automatically in hardware! */
/* Just wait for completion */
bool reached = false;
while (!reached) {
    tsumikoro_tmc5160_position_reached(&tmc, &reached);

    /* Optional: monitor progress */
    int32_t pos, vel;
    tsumikoro_tmc5160_get_position(&tmc, &pos);
    tsumikoro_tmc5160_get_velocity(&tmc, &vel);

    my_delay_us(10000);  /* 10ms */
}
```

## Integration with CMake

In your project's `CMakeLists.txt`:

```cmake
# Add TMC stepper library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared/tsumikoro_tmc_stepper
                 ${CMAKE_CURRENT_BINARY_DIR}/tsumikoro_tmc_stepper)

# Link to your executable
target_link_libraries(${PROJECT_NAME}.elf
    tsumikoro_tmc_stepper
)
```

## Choosing Between TMC2130 and TMC5160

### Use TMC2130 when:
- External microcontroller generates step pulses
- Cost-sensitive application
- Current requirements ≤ 1.4A per coil
- Existing Step/Dir motion control code

### Use TMC5160 when:
- Need integrated motion controller (less MCU load)
- Higher current capability needed (with external MOSFETs)
- Want smooth S-curve motion profiles
- Need position/velocity feedback
- Implementing complex multi-axis coordination

## References

- [TMC2130 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2130_datasheet_rev1.15.pdf)
- [TMC5160 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC5160A_datasheet_rev1.17.pdf)
- [Trinamic Motion Control](https://www.analog.com/en/parametricsearch/11236)

## License

Part of the Tsumikoro project.
