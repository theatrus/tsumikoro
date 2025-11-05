/**
 * @file tsumikoro_tmc2130.h
 * @brief TMC2130 Stepper Motor Driver Library
 *
 * Platform-independent driver for Trinamic TMC2130 stepper motor driver
 * with SPI configuration interface and Step/Dir control.
 *
 * Features:
 * - SPI register access (40-bit datagrams: 1 address + 4 data bytes)
 * - Step/Dir pin control
 * - Current configuration (IHOLD, IRUN)
 * - Microstepping configuration (256, 128, 64, 32, 16, 8, 4, 2, full step)
 * - StealthChop and SpreadCycle modes
 * - StallGuard and CoolStep support
 * - Diagnostic and status reading
 */

#ifndef TSUMIKORO_TMC2130_H
#define TSUMIKORO_TMC2130_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * TMC2130 Register Addresses
 * ============================================================================ */

/* General Configuration Registers */
#define TMC2130_REG_GCONF           0x00    /* Global configuration */
#define TMC2130_REG_GSTAT           0x01    /* Global status flags */
#define TMC2130_REG_IOIN            0x04    /* Input pin status */

/* Velocity Dependent Control Registers */
#define TMC2130_REG_IHOLD_IRUN      0x10    /* Driver current control */
#define TMC2130_REG_TPOWERDOWN      0x11    /* Delay before power down */
#define TMC2130_REG_TSTEP           0x12    /* Actual measured time between steps */
#define TMC2130_REG_TPWMTHRS        0x13    /* Upper velocity for stealthChop */
#define TMC2130_REG_TCOOLTHRS       0x14    /* Lower threshold for coolStep */
#define TMC2130_REG_THIGH           0x15    /* Velocity threshold for high speed mode */

/* Direct Mode Control */
#define TMC2130_REG_XDIRECT         0x2D    /* Direct motor coil currents */
#define TMC2130_REG_VDCMIN          0x33    /* Automatic commutation dcStep */

/* Microstepping Control Registers */
#define TMC2130_REG_MSLUT0          0x60    /* Microstep table entry 0 */
#define TMC2130_REG_MSLUT1          0x61    /* Microstep table entry 1 */
#define TMC2130_REG_MSLUT2          0x62    /* Microstep table entry 2 */
#define TMC2130_REG_MSLUT3          0x63    /* Microstep table entry 3 */
#define TMC2130_REG_MSLUT4          0x64    /* Microstep table entry 4 */
#define TMC2130_REG_MSLUT5          0x65    /* Microstep table entry 5 */
#define TMC2130_REG_MSLUT6          0x66    /* Microstep table entry 6 */
#define TMC2130_REG_MSLUT7          0x67    /* Microstep table entry 7 */
#define TMC2130_REG_MSLUTSEL        0x68    /* Microstep table segment selection */
#define TMC2130_REG_MSLUTSTART      0x69    /* Microstep table start/phase offset */
#define TMC2130_REG_MSCNT           0x6A    /* Microstep counter (read-only) */
#define TMC2130_REG_MSCURACT        0x6B    /* Actual microstep current */

/* Driver Registers */
#define TMC2130_REG_CHOPCONF        0x6C    /* Chopper configuration */
#define TMC2130_REG_COOLCONF        0x6D    /* CoolStep configuration */
#define TMC2130_REG_DCCTRL          0x6E    /* dcStep configuration */
#define TMC2130_REG_DRV_STATUS      0x6F    /* Driver status flags */
#define TMC2130_REG_PWMCONF         0x70    /* StealthChop PWM configuration */
#define TMC2130_REG_PWM_SCALE       0x71    /* StealthChop PWM scale */
#define TMC2130_REG_ENCM_CTRL       0x72    /* Encoder configuration */
#define TMC2130_REG_LOST_STEPS      0x73    /* Lost steps counter */

/* SPI Access Modes */
#define TMC2130_READ                0x00    /* Read access flag */
#define TMC2130_WRITE               0x80    /* Write access flag (bit 7) */

/* ============================================================================
 * GCONF Register Bit Definitions (0x00)
 * ============================================================================ */
#define TMC2130_GCONF_I_SCALE_ANALOG        (1 << 0)    /* Use external VREF */
#define TMC2130_GCONF_INTERNAL_RSENSE       (1 << 1)    /* Use internal sense resistors */
#define TMC2130_GCONF_EN_PWM_MODE           (1 << 2)    /* Enable stealthChop */
#define TMC2130_GCONF_ENC_COMMUTATION       (1 << 3)    /* Enable encoder commutation */
#define TMC2130_GCONF_SHAFT                 (1 << 4)    /* Inverse motor direction */
#define TMC2130_GCONF_DIAG0_ERROR           (1 << 5)    /* DIAG0 outputs driver errors */
#define TMC2130_GCONF_DIAG0_OTPW            (1 << 6)    /* DIAG0 outputs overtemp warning */
#define TMC2130_GCONF_DIAG0_STALL           (1 << 7)    /* DIAG0 outputs stallGuard */
#define TMC2130_GCONF_DIAG1_STALL           (1 << 8)    /* DIAG1 outputs stallGuard */
#define TMC2130_GCONF_DIAG1_INDEX           (1 << 9)    /* DIAG1 outputs index */
#define TMC2130_GCONF_DIAG1_ONSTATE         (1 << 10)   /* DIAG1 outputs on state */
#define TMC2130_GCONF_DIAG1_STEPS_SKIPPED   (1 << 11)   /* DIAG1 outputs steps skipped */
#define TMC2130_GCONF_DIAG0_INT_PUSHPULL    (1 << 12)   /* DIAG0 push-pull output */
#define TMC2130_GCONF_DIAG1_PUSHPULL        (1 << 13)   /* DIAG1 push-pull output */
#define TMC2130_GCONF_SMALL_HYSTERESIS      (1 << 14)   /* Small hysteresis */
#define TMC2130_GCONF_STOP_ENABLE           (1 << 15)   /* Emergency stop */
#define TMC2130_GCONF_DIRECT_MODE           (1 << 16)   /* Direct coil current mode */
#define TMC2130_GCONF_TEST_MODE             (1 << 17)   /* Test mode */

/* ============================================================================
 * CHOPCONF Register Bit Definitions (0x6C)
 * ============================================================================ */
#define TMC2130_CHOPCONF_TOFF_MASK          0x0000000F  /* Off time (NCLK = 12 + 32*TOFF) */
#define TMC2130_CHOPCONF_HSTRT_MASK         0x00000070  /* Hysteresis start (1..8) */
#define TMC2130_CHOPCONF_HEND_MASK          0x00000780  /* Hysteresis end (-3..12) */
#define TMC2130_CHOPCONF_CHM                (1 << 14)   /* Chopper mode (0=SpreadCycle, 1=constant off time) */
#define TMC2130_CHOPCONF_RNDTF              (1 << 13)   /* Random TOFF time */
#define TMC2130_CHOPCONF_TBL_MASK           0x00018000  /* Blank time (16, 24, 36, 54 clocks) */
#define TMC2130_CHOPCONF_VSENSE             (1 << 17)   /* Sense resistor voltage (0=0.325V, 1=0.180V) */
#define TMC2130_CHOPCONF_VHIGHFS            (1 << 18)   /* High velocity fullstep selection */
#define TMC2130_CHOPCONF_VHIGHCHM           (1 << 19)   /* High velocity chopper mode */
#define TMC2130_CHOPCONF_MRES_MASK          0x0F000000  /* Microstep resolution */
#define TMC2130_CHOPCONF_INTPOL             (1 << 28)   /* Interpolation to 256 microsteps */
#define TMC2130_CHOPCONF_DEDGE              (1 << 29)   /* Double edge step pulses */
#define TMC2130_CHOPCONF_DISS2G             (1 << 30)   /* Disable short to ground protection */

/* Microstep Resolution Values (for MRES field) */
#define TMC2130_MRES_256                    0           /* 256 microsteps */
#define TMC2130_MRES_128                    1           /* 128 microsteps */
#define TMC2130_MRES_64                     2           /* 64 microsteps */
#define TMC2130_MRES_32                     3           /* 32 microsteps */
#define TMC2130_MRES_16                     4           /* 16 microsteps */
#define TMC2130_MRES_8                      5           /* 8 microsteps */
#define TMC2130_MRES_4                      6           /* 4 microsteps */
#define TMC2130_MRES_2                      7           /* 2 microsteps */
#define TMC2130_MRES_FULLSTEP               8           /* Full step */

/* ============================================================================
 * DRV_STATUS Register Bit Definitions (0x6F)
 * ============================================================================ */
#define TMC2130_DRV_STATUS_SG_RESULT_MASK   0x000003FF  /* StallGuard result */
#define TMC2130_DRV_STATUS_FSACTIVE         (1 << 15)   /* Full step active */
#define TMC2130_DRV_STATUS_CS_ACTUAL_MASK   0x001F0000  /* Actual current scale */
#define TMC2130_DRV_STATUS_STALLGUARD       (1 << 24)   /* StallGuard status */
#define TMC2130_DRV_STATUS_OT               (1 << 25)   /* Overtemperature flag */
#define TMC2130_DRV_STATUS_OTPW             (1 << 26)   /* Overtemperature pre-warning */
#define TMC2130_DRV_STATUS_S2GA             (1 << 27)   /* Short to ground coil A */
#define TMC2130_DRV_STATUS_S2GB             (1 << 28)   /* Short to ground coil B */
#define TMC2130_DRV_STATUS_OLA              (1 << 29)   /* Open load coil A */
#define TMC2130_DRV_STATUS_OLB              (1 << 30)   /* Open load coil B */
#define TMC2130_DRV_STATUS_STST             (1U << 31)  /* Standstill detected */

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief TMC2130 handle structure
 *
 * Platform-specific implementation should provide SPI and GPIO callbacks
 */
typedef struct tsumikoro_tmc2130_s {
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
} tsumikoro_tmc2130_t;

/**
 * @brief Motor direction enumeration
 */
typedef enum {
    TMC2130_DIR_FORWARD = 0,
    TMC2130_DIR_REVERSE = 1
} tsumikoro_tmc2130_direction_t;

/**
 * @brief Chopper mode enumeration
 */
typedef enum {
    TMC2130_CHOPPER_SPREADCYCLE = 0,        /* SpreadCycle mode (classic) */
    TMC2130_CHOPPER_CONSTANT_TOFF = 1       /* Constant off time mode */
} tsumikoro_tmc2130_chopper_mode_t;

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize TMC2130 driver with default settings
 *
 * @param tmc Pointer to TMC2130 handle structure
 * @return true if initialization successful, false otherwise
 *
 * Note: Caller must populate SPI and GPIO callbacks before calling init
 */
bool tsumikoro_tmc2130_init(tsumikoro_tmc2130_t *tmc);

/**
 * @brief Read a TMC2130 register via SPI
 *
 * @param tmc Pointer to TMC2130 handle
 * @param reg Register address (0x00-0x7F)
 * @param value Pointer to store read value
 * @return true if read successful, false otherwise
 *
 * Note: TMC2130 returns previous register value, requires 2 transactions
 */
bool tsumikoro_tmc2130_read_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t *value);

/**
 * @brief Write a TMC2130 register via SPI
 *
 * @param tmc Pointer to TMC2130 handle
 * @param reg Register address (0x00-0x7F)
 * @param value 32-bit value to write
 * @return true if write successful, false otherwise
 */
bool tsumikoro_tmc2130_write_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t value);

/**
 * @brief Set motor direction
 *
 * @param tmc Pointer to TMC2130 handle
 * @param dir Direction (FORWARD or REVERSE)
 */
void tsumikoro_tmc2130_set_direction(tsumikoro_tmc2130_t *tmc, tsumikoro_tmc2130_direction_t dir);

/**
 * @brief Generate a single step pulse
 *
 * @param tmc Pointer to TMC2130 handle
 *
 * Note: Step pulse is generated (high->low) with internal timing
 */
void tsumikoro_tmc2130_step(tsumikoro_tmc2130_t *tmc);

/**
 * @brief Enable or disable motor driver
 *
 * @param tmc Pointer to TMC2130 handle
 * @param enable true to enable driver, false to disable
 *
 * Note: EN pin is active low (false = driver enabled)
 */
void tsumikoro_tmc2130_enable(tsumikoro_tmc2130_t *tmc, bool enable);

/**
 * @brief Set motor hold and run currents
 *
 * @param tmc Pointer to TMC2130 handle
 * @param ihold Hold current (0-31, 0=1/32, 31=32/32 of max)
 * @param irun Run current (0-31, 0=1/32, 31=32/32 of max)
 * @param iholddelay Power down delay (0-15, time = 2^18 * IHOLDDELAY / fCLK)
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc2130_set_current(tsumikoro_tmc2130_t *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay);

/**
 * @brief Set microstep resolution
 *
 * @param tmc Pointer to TMC2130 handle
 * @param mres Microstep resolution (TMC2130_MRES_xxx)
 * @param interpolate Enable 256 microstep interpolation
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc2130_set_microsteps(tsumikoro_tmc2130_t *tmc, uint8_t mres, bool interpolate);

/**
 * @brief Enable or disable stealthChop mode
 *
 * @param tmc Pointer to TMC2130 handle
 * @param enable true to enable stealthChop, false for SpreadCycle
 * @return true if successful, false otherwise
 */
bool tsumikoro_tmc2130_set_stealth_chop(tsumikoro_tmc2130_t *tmc, bool enable);

/**
 * @brief Read driver status register
 *
 * @param tmc Pointer to TMC2130 handle
 * @param status Pointer to store status value
 * @return true if read successful, false otherwise
 */
bool tsumikoro_tmc2130_read_status(tsumikoro_tmc2130_t *tmc, uint32_t *status);

/**
 * @brief Check if motor is in standstill
 *
 * @param tmc Pointer to TMC2130 handle
 * @param standstill Pointer to store standstill flag
 * @return true if read successful, false otherwise
 */
bool tsumikoro_tmc2130_is_standstill(tsumikoro_tmc2130_t *tmc, bool *standstill);

/**
 * @brief Check for driver errors
 *
 * @param tmc Pointer to TMC2130 handle
 * @param has_error Pointer to store error flag
 * @return true if read successful, false otherwise
 *
 * Checks for: overtemperature, short to ground, open load
 */
bool tsumikoro_tmc2130_has_error(tsumikoro_tmc2130_t *tmc, bool *has_error);

/**
 * @brief Get StallGuard result value
 *
 * @param tmc Pointer to TMC2130 handle
 * @param result Pointer to store StallGuard result (0-1023)
 * @return true if read successful, false otherwise
 *
 * Higher values = lower motor load. Use for sensorless homing.
 */
bool tsumikoro_tmc2130_get_stallguard(tsumikoro_tmc2130_t *tmc, uint16_t *result);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_TMC2130_H */
