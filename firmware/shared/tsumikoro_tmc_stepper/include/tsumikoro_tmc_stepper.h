/**
 * @file tsumikoro_tmc_stepper.h
 * @brief Trinamic TMC2130/TMC5160 Stepper Motor Driver Library
 *
 * Platform-independent driver for Trinamic stepper motor drivers:
 * - TMC2130: Driver with Step/Dir control
 * - TMC5160: Driver with integrated motion controller
 *
 * Features:
 * - SPI register access (40-bit datagrams: 1 address + 4 data bytes)
 * - Step/Dir pin control (both chips)
 * - Current configuration (IHOLD, IRUN)
 * - Microstepping configuration (256, 128, 64, 32, 16, 8, 4, 2, full step)
 * - StealthChop and SpreadCycle modes
 * - StallGuard and CoolStep support
 * - Diagnostic and status reading
 * - TMC5160: Integrated 6-point ramp generator for autonomous motion
 */

#ifndef TSUMIKORO_TMC_STEPPER_H
#define TSUMIKORO_TMC_STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Shared Register Addresses (TMC2130 and TMC5160)
 * ============================================================================ */

/* General Configuration Registers */
#define TMC_REG_GCONF           0x00    /* Global configuration */
#define TMC_REG_GSTAT           0x01    /* Global status flags */

/* Velocity Dependent Control Registers */
#define TMC_REG_IHOLD_IRUN      0x10    /* Driver current control */
#define TMC_REG_TPOWERDOWN      0x11    /* Delay before power down */
#define TMC_REG_TSTEP           0x12    /* Actual measured time between steps */
#define TMC_REG_TPWMTHRS        0x13    /* Upper velocity for stealthChop */
#define TMC_REG_TCOOLTHRS       0x14    /* Lower threshold for coolStep */
#define TMC_REG_THIGH           0x15    /* Velocity threshold for high speed mode */

/* Microstepping Control Registers */
#define TMC_REG_MSLUT0          0x60    /* Microstep table entry 0 */
#define TMC_REG_MSLUT1          0x61    /* Microstep table entry 1 */
#define TMC_REG_MSLUT2          0x62    /* Microstep table entry 2 */
#define TMC_REG_MSLUT3          0x63    /* Microstep table entry 3 */
#define TMC_REG_MSLUT4          0x64    /* Microstep table entry 4 */
#define TMC_REG_MSLUT5          0x65    /* Microstep table entry 5 */
#define TMC_REG_MSLUT6          0x66    /* Microstep table entry 6 */
#define TMC_REG_MSLUT7          0x67    /* Microstep table entry 7 */
#define TMC_REG_MSLUTSEL        0x68    /* Microstep table segment selection */
#define TMC_REG_MSLUTSTART      0x69    /* Microstep table start/phase offset */
#define TMC_REG_MSCNT           0x6A    /* Microstep counter (read-only) */
#define TMC_REG_MSCURACT        0x6B    /* Actual microstep current */

/* Driver Registers */
#define TMC_REG_CHOPCONF        0x6C    /* Chopper configuration */
#define TMC_REG_COOLCONF        0x6D    /* CoolStep configuration */
#define TMC_REG_DCCTRL          0x6E    /* dcStep configuration */
#define TMC_REG_DRV_STATUS      0x6F    /* Driver status flags */
#define TMC_REG_PWMCONF         0x70    /* StealthChop PWM configuration */
#define TMC_REG_PWM_SCALE       0x71    /* StealthChop PWM scale */
#define TMC_REG_LOST_STEPS      0x73    /* Lost steps counter */

/* ============================================================================
 * TMC2130-Specific Registers
 * ============================================================================ */
#define TMC2130_REG_IOIN        0x04    /* Input pin status */
#define TMC2130_REG_XDIRECT     0x2D    /* Direct motor coil currents */
#define TMC2130_REG_ENCM_CTRL   0x72    /* Encoder configuration */

/* ============================================================================
 * TMC5160-Specific Registers (Motion Controller)
 * ============================================================================ */

/* Ramp Generator Registers */
#define TMC5160_REG_RAMPMODE    0x20    /* Ramp generator mode */
#define TMC5160_REG_XACTUAL     0x21    /* Actual motor position */
#define TMC5160_REG_VACTUAL     0x22    /* Actual motor velocity */
#define TMC5160_REG_VSTART      0x23    /* Motor start velocity */
#define TMC5160_REG_A1          0x24    /* First acceleration */
#define TMC5160_REG_V1          0x25    /* First acceleration threshold */
#define TMC5160_REG_AMAX        0x26    /* Maximum acceleration */
#define TMC5160_REG_VMAX        0x27    /* Maximum velocity */
#define TMC5160_REG_DMAX        0x28    /* Maximum deceleration */
#define TMC5160_REG_D1          0x2A    /* First deceleration */
#define TMC5160_REG_VSTOP       0x2B    /* Motor stop velocity */
#define TMC5160_REG_TZEROWAIT   0x2C    /* Waiting time after ramp down to zero */
#define TMC5160_REG_XTARGET     0x2D    /* Target position */

/* Status Registers */
#define TMC5160_REG_VDCMIN      0x33    /* DC step minimum velocity */
#define TMC5160_REG_SW_MODE     0x34    /* Switch mode configuration */
#define TMC5160_REG_RAMP_STAT   0x35    /* Ramp status flags */
#define TMC5160_REG_XLATCH      0x36    /* Latched position on event */

/* SPI Access Modes */
#define TMC_READ                0x00    /* Read access flag */
#define TMC_WRITE               0x80    /* Write access flag (bit 7) */

/* ============================================================================
 * GCONF Register Bit Definitions (0x00)
 * ============================================================================ */
#define TMC_GCONF_I_SCALE_ANALOG        (1 << 0)    /* Use external VREF */
#define TMC_GCONF_INTERNAL_RSENSE       (1 << 1)    /* Use internal sense resistors */
#define TMC_GCONF_EN_PWM_MODE           (1 << 2)    /* Enable stealthChop */
#define TMC_GCONF_ENC_COMMUTATION       (1 << 3)    /* Enable encoder commutation */
#define TMC_GCONF_SHAFT                 (1 << 4)    /* Inverse motor direction */
#define TMC_GCONF_DIAG0_ERROR           (1 << 5)    /* DIAG0 outputs driver errors */
#define TMC_GCONF_DIAG0_OTPW            (1 << 6)    /* DIAG0 outputs overtemp warning */
#define TMC_GCONF_DIAG0_STALL           (1 << 7)    /* DIAG0 outputs stallGuard */
#define TMC_GCONF_DIAG1_STALL           (1 << 8)    /* DIAG1 outputs stallGuard */
#define TMC_GCONF_DIAG1_INDEX           (1 << 9)    /* DIAG1 outputs index */
#define TMC_GCONF_DIAG1_ONSTATE         (1 << 10)   /* DIAG1 outputs on state */
#define TMC_GCONF_DIAG1_STEPS_SKIPPED   (1 << 11)   /* DIAG1 outputs steps skipped */
#define TMC_GCONF_DIAG0_INT_PUSHPULL    (1 << 12)   /* DIAG0 push-pull output */
#define TMC_GCONF_DIAG1_PUSHPULL        (1 << 13)   /* DIAG1 push-pull output */
#define TMC_GCONF_SMALL_HYSTERESIS      (1 << 14)   /* Small hysteresis */
#define TMC_GCONF_STOP_ENABLE           (1 << 15)   /* Emergency stop */
#define TMC_GCONF_DIRECT_MODE           (1 << 16)   /* Direct coil current mode */
#define TMC_GCONF_TEST_MODE             (1 << 17)   /* Test mode */

/* ============================================================================
 * CHOPCONF Register Bit Definitions (0x6C)
 * ============================================================================ */
#define TMC_CHOPCONF_TOFF_MASK          0x0000000F  /* Off time (NCLK = 12 + 32*TOFF) */
#define TMC_CHOPCONF_HSTRT_MASK         0x00000070  /* Hysteresis start (1..8) */
#define TMC_CHOPCONF_HEND_MASK          0x00000780  /* Hysteresis end (-3..12) */
#define TMC_CHOPCONF_CHM                (1 << 14)   /* Chopper mode (0=SpreadCycle, 1=constant off time) */
#define TMC_CHOPCONF_RNDTF              (1 << 13)   /* Random TOFF time */
#define TMC_CHOPCONF_TBL_MASK           0x00018000  /* Blank time (16, 24, 36, 54 clocks) */
#define TMC_CHOPCONF_VSENSE             (1 << 17)   /* Sense resistor voltage (0=0.325V, 1=0.180V) */
#define TMC_CHOPCONF_VHIGHFS            (1 << 18)   /* High velocity fullstep selection */
#define TMC_CHOPCONF_VHIGHCHM           (1 << 19)   /* High velocity chopper mode */
#define TMC_CHOPCONF_MRES_MASK          0x0F000000  /* Microstep resolution */
#define TMC_CHOPCONF_INTPOL             (1 << 28)   /* Interpolation to 256 microsteps */
#define TMC_CHOPCONF_DEDGE              (1 << 29)   /* Double edge step pulses */
#define TMC_CHOPCONF_DISS2G             (1 << 30)   /* Disable short to ground protection */

/* Microstep Resolution Values (for MRES field) */
#define TMC_MRES_256                    0           /* 256 microsteps */
#define TMC_MRES_128                    1           /* 128 microsteps */
#define TMC_MRES_64                     2           /* 64 microsteps */
#define TMC_MRES_32                     3           /* 32 microsteps */
#define TMC_MRES_16                     4           /* 16 microsteps */
#define TMC_MRES_8                      5           /* 8 microsteps */
#define TMC_MRES_4                      6           /* 4 microsteps */
#define TMC_MRES_2                      7           /* 2 microsteps */
#define TMC_MRES_FULLSTEP               8           /* Full step */

/* ============================================================================
 * DRV_STATUS Register Bit Definitions (0x6F)
 * ============================================================================ */
#define TMC_DRV_STATUS_SG_RESULT_MASK   0x000003FF  /* StallGuard result */
#define TMC_DRV_STATUS_FSACTIVE         (1 << 15)   /* Full step active */
#define TMC_DRV_STATUS_CS_ACTUAL_MASK   0x001F0000  /* Actual current scale */
#define TMC_DRV_STATUS_STALLGUARD       (1 << 24)   /* StallGuard status */
#define TMC_DRV_STATUS_OT               (1 << 25)   /* Overtemperature flag */
#define TMC_DRV_STATUS_OTPW             (1 << 26)   /* Overtemperature pre-warning */
#define TMC_DRV_STATUS_S2GA             (1 << 27)   /* Short to ground coil A */
#define TMC_DRV_STATUS_S2GB             (1 << 28)   /* Short to ground coil B */
#define TMC_DRV_STATUS_OLA              (1 << 29)   /* Open load coil A */
#define TMC_DRV_STATUS_OLB              (1 << 30)   /* Open load coil B */
#define TMC_DRV_STATUS_STST             (1U << 31)  /* Standstill detected */

/* ============================================================================
 * TMC5160 RAMPMODE Register Values (0x20)
 * ============================================================================ */
#define TMC5160_RAMPMODE_POSITIONING    0           /* Positioning mode (move to XTARGET) */
#define TMC5160_RAMPMODE_VELOCITY_POS   1           /* Velocity mode (positive VMAX) */
#define TMC5160_RAMPMODE_VELOCITY_NEG   2           /* Velocity mode (negative VMAX) */
#define TMC5160_RAMPMODE_HOLD           3           /* Hold mode (velocity = 0) */

/* ============================================================================
 * TMC5160 RAMP_STAT Register Bit Definitions (0x35)
 * ============================================================================ */
#define TMC5160_RAMP_STAT_STATUS_STOP_L         (1 << 0)    /* Left stop switch active */
#define TMC5160_RAMP_STAT_STATUS_STOP_R         (1 << 1)    /* Right stop switch active */
#define TMC5160_RAMP_STAT_STATUS_LATCH_L        (1 << 2)    /* Left stop switch latched */
#define TMC5160_RAMP_STAT_STATUS_LATCH_R        (1 << 3)    /* Right stop switch latched */
#define TMC5160_RAMP_STAT_EVENT_STOP_L          (1 << 4)    /* Left stop event occurred */
#define TMC5160_RAMP_STAT_EVENT_STOP_R          (1 << 5)    /* Right stop event occurred */
#define TMC5160_RAMP_STAT_EVENT_STOP_SG         (1 << 6)    /* StallGuard stop event */
#define TMC5160_RAMP_STAT_EVENT_POS_REACHED     (1 << 7)    /* Target position reached */
#define TMC5160_RAMP_STAT_VELOCITY_REACHED      (1 << 8)    /* Target velocity reached */
#define TMC5160_RAMP_STAT_POSITION_REACHED      (1 << 9)    /* Position reached and stable */
#define TMC5160_RAMP_STAT_VZERO                 (1 << 10)   /* Velocity is zero */
#define TMC5160_RAMP_STAT_T_ZEROWAIT_ACTIVE     (1 << 11)   /* TZEROWAIT timer active */
#define TMC5160_RAMP_STAT_SECOND_MOVE           (1 << 12)   /* Second move ongoing */
#define TMC5160_RAMP_STAT_STATUS_SG             (1 << 13)   /* StallGuard active */

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief TMC chip type enumeration
 */
typedef enum {
    TMC_CHIP_TMC2130,       /* TMC2130: Step/Dir driver */
    TMC_CHIP_TMC5160        /* TMC5160: Driver with motion controller */
} tsumikoro_tmc_chip_type_t;

/**
 * @brief TMC stepper driver handle structure
 *
 * Platform-specific implementation should provide SPI and GPIO callbacks
 */
typedef struct tsumikoro_tmc_stepper_s {
    /* Chip type */
    tsumikoro_tmc_chip_type_t chip_type;

    /* SPI communication callbacks (must be provided by platform) */
    void (*spi_begin)(void);                            /* Assert CS pin */
    void (*spi_end)(void);                              /* Deassert CS pin */
    uint8_t (*spi_transfer)(uint8_t data);              /* Transfer single byte */

    /* GPIO control callbacks (must be provided by platform) */
    void (*set_step_pin)(bool state);                   /* Set STEP pin state */
    void (*set_dir_pin)(bool state);                    /* Set DIR pin state */
    void (*set_enable_pin)(bool state);                 /* Set EN pin state (active low) */

    /* Optional delay function (microseconds) */
    void (*delay_us)(uint32_t us);

    /* Cached register values */
    uint32_t gconf;
    uint32_t chopconf;
    uint32_t ihold_irun;
    uint32_t coolconf;
    uint32_t pwmconf;
} tsumikoro_tmc_stepper_t;

/**
 * @brief Motor direction enumeration
 */
typedef enum {
    TMC_DIR_FORWARD = 0,
    TMC_DIR_REVERSE = 1
} tsumikoro_tmc_direction_t;

/**
 * @brief Chopper mode enumeration
 */
typedef enum {
    TMC_CHOPPER_SPREADCYCLE = 0,        /* SpreadCycle mode (classic) */
    TMC_CHOPPER_CONSTANT_TOFF = 1       /* Constant off time mode */
} tsumikoro_tmc_chopper_mode_t;

/* ============================================================================
 * Common API Functions (TMC2130 and TMC5160)
 * ============================================================================ */

/**
 * @brief Initialize TMC driver with default settings
 *
 * @param tmc Pointer to TMC handle structure
 * @param chip_type Chip type (TMC_CHIP_TMC2130 or TMC_CHIP_TMC5160)
 * @return true if initialization successful, false otherwise
 *
 * Note: Caller must populate SPI and GPIO callbacks before calling init
 */
bool tsumikoro_tmc_init(tsumikoro_tmc_stepper_t *tmc, tsumikoro_tmc_chip_type_t chip_type);

/**
 * @brief Read a TMC register via SPI
 *
 * @param tmc Pointer to TMC handle
 * @param reg Register address (0x00-0x7F)
 * @param value Pointer to store read value
 * @return true if read successful, false otherwise
 *
 * Note: TMC chips return previous register value, requires 2 transactions
 */
bool tsumikoro_tmc_read_register(tsumikoro_tmc_stepper_t *tmc, uint8_t reg, uint32_t *value);

/**
 * @brief Write a TMC register via SPI
 *
 * @param tmc Pointer to TMC handle
 * @param reg Register address (0x00-0x7F)
 * @param value 32-bit value to write
 * @return true if write successful, false otherwise
 */
bool tsumikoro_tmc_write_register(tsumikoro_tmc_stepper_t *tmc, uint8_t reg, uint32_t value);

/**
 * @brief Set motor direction
 *
 * @param tmc Pointer to TMC handle
 * @param dir Direction (FORWARD or REVERSE)
 */
void tsumikoro_tmc_set_direction(tsumikoro_tmc_stepper_t *tmc, tsumikoro_tmc_direction_t dir);

/**
 * @brief Generate a single step pulse
 *
 * @param tmc Pointer to TMC handle
 *
 * Note: Step pulse is generated (high->low) with internal timing
 */
void tsumikoro_tmc_step(tsumikoro_tmc_stepper_t *tmc);

/**
 * @brief Enable or disable motor driver
 *
 * @param tmc Pointer to TMC handle
 * @param enable true to enable driver, false to disable
 *
 * Note: EN pin is active low (false = driver enabled)
 */
void tsumikoro_tmc_enable(tsumikoro_tmc_stepper_t *tmc, bool enable);

/**
 * @brief Set motor hold and run currents
 *
 * @param tmc Pointer to TMC handle
 * @param ihold Hold current (0-31, 0=1/32, 31=32/32 of max)
 * @param irun Run current (0-31, 0=1/32, 31=32/32 of max)
 * @param iholddelay Power down delay (0-15, time = 2^18 * IHOLDDELAY / fCLK)
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc_set_current(tsumikoro_tmc_stepper_t *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay);

/**
 * @brief Set microstep resolution
 *
 * @param tmc Pointer to TMC handle
 * @param mres Microstep resolution (TMC_MRES_xxx)
 * @param interpolate Enable 256 microstep interpolation
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc_set_microsteps(tsumikoro_tmc_stepper_t *tmc, uint8_t mres, bool interpolate);

/**
 * @brief Enable or disable stealthChop mode
 *
 * @param tmc Pointer to TMC handle
 * @param enable true to enable stealthChop, false for SpreadCycle
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc_set_stealth_chop(tsumikoro_tmc_stepper_t *tmc, bool enable);

/**
 * @brief Read driver status register
 *
 * @param tmc Pointer to TMC handle
 * @param status Pointer to store status value
 * @return true if read successful, false otherwise
 */
bool tsumikoro_tmc_read_status(tsumikoro_tmc_stepper_t *tmc, uint32_t *status);

/**
 * @brief Check if motor is in standstill
 *
 * @param tmc Pointer to TMC handle
 * @param standstill Pointer to store standstill flag
 * @return true if read successful, false otherwise
 */
bool tsumikoro_tmc_is_standstill(tsumikoro_tmc_stepper_t *tmc, bool *standstill);

/**
 * @brief Check for driver errors
 *
 * @param tmc Pointer to TMC handle
 * @param has_error Pointer to store error flag
 * @return true if read successful, false otherwise
 *
 * Checks for: overtemperature, short to ground, open load
 */
bool tsumikoro_tmc_has_error(tsumikoro_tmc_stepper_t *tmc, bool *has_error);

/**
 * @brief Get StallGuard result value
 *
 * @param tmc Pointer to TMC handle
 * @param result Pointer to store StallGuard result (0-1023)
 * @return true if read successful, false otherwise
 *
 * Higher values = lower motor load. Use for sensorless homing.
 */
bool tsumikoro_tmc_get_stallguard(tsumikoro_tmc_stepper_t *tmc, uint16_t *result);

/* ============================================================================
 * TMC5160-Specific API Functions (Motion Controller)
 * ============================================================================ */

/**
 * @brief Set ramp mode for TMC5160 motion controller
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param mode Ramp mode (TMC5160_RAMPMODE_xxx)
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_set_ramp_mode(tsumikoro_tmc_stepper_t *tmc, uint8_t mode);

/**
 * @brief Set TMC5160 ramp parameters
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param vstart Starting velocity
 * @param a1 First acceleration
 * @param v1 First acceleration/deceleration threshold
 * @param amax Maximum acceleration
 * @param vmax Maximum velocity
 * @param dmax Maximum deceleration
 * @param d1 First deceleration
 * @param vstop Stop velocity
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_set_ramp_params(tsumikoro_tmc_stepper_t *tmc,
                                       uint32_t vstart, uint32_t a1, uint32_t v1,
                                       uint32_t amax, uint32_t vmax,
                                       uint32_t dmax, uint32_t d1, uint32_t vstop);

/**
 * @brief Move to absolute position (positioning mode)
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param position Target position (signed 32-bit)
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_move_to(tsumikoro_tmc_stepper_t *tmc, int32_t position);

/**
 * @brief Get current position
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param position Pointer to store current position
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_get_position(tsumikoro_tmc_stepper_t *tmc, int32_t *position);

/**
 * @brief Set current position (without moving)
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param position New position value
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_set_position(tsumikoro_tmc_stepper_t *tmc, int32_t position);

/**
 * @brief Get current velocity
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param velocity Pointer to store current velocity
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_get_velocity(tsumikoro_tmc_stepper_t *tmc, int32_t *velocity);

/**
 * @brief Check if target position has been reached
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @param reached Pointer to store reached flag
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_position_reached(tsumikoro_tmc_stepper_t *tmc, bool *reached);

/**
 * @brief Stop motor immediately
 *
 * @param tmc Pointer to TMC handle (must be TMC5160)
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc5160_stop(tsumikoro_tmc_stepper_t *tmc);

/* ============================================================================
 * Legacy TMC2130 API (for backward compatibility)
 * ============================================================================ */

/* Type aliases for backward compatibility */
typedef tsumikoro_tmc_stepper_t tsumikoro_tmc2130_t;
typedef tsumikoro_tmc_direction_t tsumikoro_tmc2130_direction_t;
typedef tsumikoro_tmc_chopper_mode_t tsumikoro_tmc2130_chopper_mode_t;

#define TMC2130_DIR_FORWARD     TMC_DIR_FORWARD
#define TMC2130_DIR_REVERSE     TMC_DIR_REVERSE
#define TMC2130_CHOPPER_SPREADCYCLE     TMC_CHOPPER_SPREADCYCLE
#define TMC2130_CHOPPER_CONSTANT_TOFF   TMC_CHOPPER_CONSTANT_TOFF

/* Legacy function wrappers */
static inline bool tsumikoro_tmc2130_init(tsumikoro_tmc2130_t *tmc) {
    return tsumikoro_tmc_init(tmc, TMC_CHIP_TMC2130);
}

static inline bool tsumikoro_tmc2130_read_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t *value) {
    return tsumikoro_tmc_read_register(tmc, reg, value);
}

static inline bool tsumikoro_tmc2130_write_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t value) {
    return tsumikoro_tmc_write_register(tmc, reg, value);
}

static inline void tsumikoro_tmc2130_set_direction(tsumikoro_tmc2130_t *tmc, tsumikoro_tmc2130_direction_t dir) {
    tsumikoro_tmc_set_direction(tmc, dir);
}

static inline void tsumikoro_tmc2130_step(tsumikoro_tmc2130_t *tmc) {
    tsumikoro_tmc_step(tmc);
}

static inline void tsumikoro_tmc2130_enable(tsumikoro_tmc2130_t *tmc, bool enable) {
    tsumikoro_tmc_enable(tmc, enable);
}

static inline bool tsumikoro_tmc2130_set_current(tsumikoro_tmc2130_t *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
    return tsumikoro_tmc_set_current(tmc, ihold, irun, iholddelay);
}

static inline bool tsumikoro_tmc2130_set_microsteps(tsumikoro_tmc2130_t *tmc, uint8_t mres, bool interpolate) {
    return tsumikoro_tmc_set_microsteps(tmc, mres, interpolate);
}

static inline bool tsumikoro_tmc2130_set_stealth_chop(tsumikoro_tmc2130_t *tmc, bool enable) {
    return tsumikoro_tmc_set_stealth_chop(tmc, enable);
}

static inline bool tsumikoro_tmc2130_read_status(tsumikoro_tmc2130_t *tmc, uint32_t *status) {
    return tsumikoro_tmc_read_status(tmc, status);
}

static inline bool tsumikoro_tmc2130_is_standstill(tsumikoro_tmc2130_t *tmc, bool *standstill) {
    return tsumikoro_tmc_is_standstill(tmc, standstill);
}

static inline bool tsumikoro_tmc2130_has_error(tsumikoro_tmc2130_t *tmc, bool *has_error) {
    return tsumikoro_tmc_has_error(tmc, has_error);
}

static inline bool tsumikoro_tmc2130_get_stallguard(tsumikoro_tmc2130_t *tmc, uint16_t *result) {
    return tsumikoro_tmc_get_stallguard(tmc, result);
}

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_TMC_STEPPER_H */
