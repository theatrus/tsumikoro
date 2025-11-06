/**
 * @file tsumikoro_tmc_stepper.c
 * @brief Trinamic TMC2130/TMC5160 Stepper Motor Driver Implementation
 */

#include "tsumikoro_tmc_stepper.h"
#include <string.h>

/* Default register values for initialization */
#define TMC_DEFAULT_GCONF       (0x00000000)
#define TMC_DEFAULT_CHOPCONF    (0x00008008)  /* TOFF=8, HSTRT=0, HEND=0 */
#define TMC_DEFAULT_IHOLD_IRUN  (0x00001F00)  /* IHOLD=0, IRUN=31, IHOLDDELAY=0 */
#define TMC_DEFAULT_TPOWERDOWN  (0x0000000A)  /* ~1 second at 12MHz */
#define TMC_DEFAULT_TPWMTHRS    (0x000001F4)  /* ~500 */
#define TMC_DEFAULT_PWMCONF     (0x000401C8)  /* Default PWM config */
#define TMC_DEFAULT_COOLCONF    (0x00000000)

/* TMC5160-specific defaults */
#define TMC5160_DEFAULT_VSTART  (0x00000001)
#define TMC5160_DEFAULT_A1      (0x000003E8)  /* 1000 */
#define TMC5160_DEFAULT_V1      (0x00007530)  /* 30000 */
#define TMC5160_DEFAULT_AMAX    (0x000003E8)  /* 1000 */
#define TMC5160_DEFAULT_VMAX    (0x0000C350)  /* 50000 */
#define TMC5160_DEFAULT_DMAX    (0x000003E8)  /* 1000 */
#define TMC5160_DEFAULT_D1      (0x000003E8)  /* 1000 */
#define TMC5160_DEFAULT_VSTOP   (0x0000000A)  /* 10 */

/* Timing constants (microseconds) */
#define TMC_SPI_DELAY_US        1
#define TMC_STEP_PULSE_US       2
#define TMC_STEP_PAUSE_US       2

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Transfer 40-bit SPI datagram (1 address + 4 data bytes)
 *
 * @param tmc Pointer to TMC handle
 * @param addr_rw Address byte with read/write flag
 * @param data_out Data to write (32-bit)
 * @param data_in Pointer to store read data (32-bit)
 */
static void tmc_spi_transfer_datagram(tsumikoro_tmc_stepper_t *tmc, uint8_t addr_rw,
                                      uint32_t data_out, uint32_t *data_in)
{
    if (!tmc || !tmc->spi_begin || !tmc->spi_end || !tmc->spi_transfer) {
        return;
    }

    tmc->spi_begin();

    /* Send address byte */
    tmc->spi_transfer(addr_rw);

    /* Send/receive data bytes (MSB first) */
    uint32_t received = 0;
    received |= ((uint32_t)tmc->spi_transfer((data_out >> 24) & 0xFF)) << 24;
    received |= ((uint32_t)tmc->spi_transfer((data_out >> 16) & 0xFF)) << 16;
    received |= ((uint32_t)tmc->spi_transfer((data_out >> 8) & 0xFF)) << 8;
    received |= ((uint32_t)tmc->spi_transfer(data_out & 0xFF));

    tmc->spi_end();

    if (data_in) {
        *data_in = received;
    }

    /* Small delay between transactions */
    if (tmc->delay_us) {
        tmc->delay_us(TMC_SPI_DELAY_US);
    }
}

/* ============================================================================
 * Common API Implementation (TMC2130 and TMC5160)
 * ============================================================================ */

bool tsumikoro_tmc_init(tsumikoro_tmc_stepper_t *tmc, tsumikoro_tmc_chip_type_t chip_type)
{
    if (!tmc) {
        return false;
    }

    /* Verify required callbacks are provided */
    if (!tmc->spi_begin || !tmc->spi_end || !tmc->spi_transfer) {
        return false;
    }

    /* Set chip type */
    tmc->chip_type = chip_type;

    /* Initialize cached register values */
    tmc->gconf = TMC_DEFAULT_GCONF;
    tmc->chopconf = TMC_DEFAULT_CHOPCONF;
    tmc->ihold_irun = TMC_DEFAULT_IHOLD_IRUN;
    tmc->coolconf = TMC_DEFAULT_COOLCONF;
    tmc->pwmconf = TMC_DEFAULT_PWMCONF;

    /* Configure default registers (common to both chips) */
    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_GCONF, tmc->gconf)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_CHOPCONF, tmc->chopconf)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_IHOLD_IRUN, tmc->ihold_irun)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_TPOWERDOWN, TMC_DEFAULT_TPOWERDOWN)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_TPWMTHRS, TMC_DEFAULT_TPWMTHRS)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_PWMCONF, tmc->pwmconf)) {
        return false;
    }

    if (!tsumikoro_tmc_write_register(tmc, TMC_REG_COOLCONF, tmc->coolconf)) {
        return false;
    }

    /* TMC5160-specific initialization */
    if (chip_type == TMC_CHIP_TMC5160) {
        /* Initialize ramp generator with safe defaults */
        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITIONING)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VSTART, TMC5160_DEFAULT_VSTART)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_A1, TMC5160_DEFAULT_A1)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_V1, TMC5160_DEFAULT_V1)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_AMAX, TMC5160_DEFAULT_AMAX)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VMAX, TMC5160_DEFAULT_VMAX)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_DMAX, TMC5160_DEFAULT_DMAX)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_D1, TMC5160_DEFAULT_D1)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VSTOP, TMC5160_DEFAULT_VSTOP)) {
            return false;
        }

        /* Set initial position to 0 */
        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_XACTUAL, 0)) {
            return false;
        }

        if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_XTARGET, 0)) {
            return false;
        }
    }

    return true;
}

bool tsumikoro_tmc_read_register(tsumikoro_tmc_stepper_t *tmc, uint8_t reg, uint32_t *value)
{
    if (!tmc || !value || reg > 0x7F) {
        return false;
    }

    /* TMC chips return data from PREVIOUS read operation
     * Need to issue read twice: once to request, once to receive */

    uint32_t dummy = 0;

    /* First read: request the register */
    tmc_spi_transfer_datagram(tmc, reg | TMC_READ, 0x00000000, &dummy);

    /* Second read: receive the data (send dummy read to same register) */
    tmc_spi_transfer_datagram(tmc, reg | TMC_READ, 0x00000000, value);

    return true;
}

bool tsumikoro_tmc_write_register(tsumikoro_tmc_stepper_t *tmc, uint8_t reg, uint32_t value)
{
    if (!tmc || reg > 0x7F) {
        return false;
    }

    /* Write operation */
    tmc_spi_transfer_datagram(tmc, reg | TMC_WRITE, value, NULL);

    /* Update cached value for frequently accessed registers */
    switch (reg) {
        case TMC_REG_GCONF:
            tmc->gconf = value;
            break;
        case TMC_REG_CHOPCONF:
            tmc->chopconf = value;
            break;
        case TMC_REG_IHOLD_IRUN:
            tmc->ihold_irun = value;
            break;
        case TMC_REG_COOLCONF:
            tmc->coolconf = value;
            break;
        case TMC_REG_PWMCONF:
            tmc->pwmconf = value;
            break;
        default:
            break;
    }

    return true;
}

void tsumikoro_tmc_set_direction(tsumikoro_tmc_stepper_t *tmc, tsumikoro_tmc_direction_t dir)
{
    if (!tmc || !tmc->set_dir_pin) {
        return;
    }

    tmc->set_dir_pin(dir == TMC_DIR_FORWARD ? false : true);
}

void tsumikoro_tmc_step(tsumikoro_tmc_stepper_t *tmc)
{
    if (!tmc || !tmc->set_step_pin) {
        return;
    }

    /* Generate step pulse: low->high->low */
    tmc->set_step_pin(true);

    if (tmc->delay_us) {
        tmc->delay_us(TMC_STEP_PULSE_US);
    }

    tmc->set_step_pin(false);

    if (tmc->delay_us) {
        tmc->delay_us(TMC_STEP_PAUSE_US);
    }
}

void tsumikoro_tmc_enable(tsumikoro_tmc_stepper_t *tmc, bool enable)
{
    if (!tmc || !tmc->set_enable_pin) {
        return;
    }

    /* EN pin is active low: false = enabled, true = disabled */
    tmc->set_enable_pin(!enable);
}

bool tsumikoro_tmc_set_current(tsumikoro_tmc_stepper_t *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    if (!tmc) {
        return false;
    }

    /* Clamp values to valid ranges */
    if (ihold > 31) ihold = 31;
    if (irun > 31) irun = 31;
    if (iholddelay > 15) iholddelay = 15;

    /* Build IHOLD_IRUN register value
     * Bits [4:0]   = IHOLD (standstill current)
     * Bits [12:8]  = IRUN (run current)
     * Bits [19:16] = IHOLDDELAY (power down delay) */
    uint32_t value = ((uint32_t)ihold & 0x1F) |
                     (((uint32_t)irun & 0x1F) << 8) |
                     (((uint32_t)iholddelay & 0x0F) << 16);

    return tsumikoro_tmc_write_register(tmc, TMC_REG_IHOLD_IRUN, value);
}

bool tsumikoro_tmc_set_microsteps(tsumikoro_tmc_stepper_t *tmc, uint8_t mres, bool interpolate)
{
    if (!tmc) {
        return false;
    }

    /* Clamp mres to valid range (0-8) */
    if (mres > TMC_MRES_FULLSTEP) {
        mres = TMC_MRES_FULLSTEP;
    }

    /* Modify CHOPCONF register
     * Bits [27:24] = MRES (microstep resolution)
     * Bit 28       = INTPOL (interpolation enable) */
    uint32_t chopconf = tmc->chopconf;

    /* Clear MRES and INTPOL bits */
    chopconf &= ~(TMC_CHOPCONF_MRES_MASK | TMC_CHOPCONF_INTPOL);

    /* Set new MRES value */
    chopconf |= ((uint32_t)mres << 24);

    /* Set interpolation if requested */
    if (interpolate) {
        chopconf |= TMC_CHOPCONF_INTPOL;
    }

    return tsumikoro_tmc_write_register(tmc, TMC_REG_CHOPCONF, chopconf);
}

bool tsumikoro_tmc_set_stealth_chop(tsumikoro_tmc_stepper_t *tmc, bool enable)
{
    if (!tmc) {
        return false;
    }

    /* Modify GCONF register
     * Bit 2 = EN_PWM_MODE (stealthChop enable) */
    uint32_t gconf = tmc->gconf;

    if (enable) {
        gconf |= TMC_GCONF_EN_PWM_MODE;
    } else {
        gconf &= ~TMC_GCONF_EN_PWM_MODE;
    }

    return tsumikoro_tmc_write_register(tmc, TMC_REG_GCONF, gconf);
}

bool tsumikoro_tmc_read_status(tsumikoro_tmc_stepper_t *tmc, uint32_t *status)
{
    if (!tmc || !status) {
        return false;
    }

    return tsumikoro_tmc_read_register(tmc, TMC_REG_DRV_STATUS, status);
}

bool tsumikoro_tmc_is_standstill(tsumikoro_tmc_stepper_t *tmc, bool *standstill)
{
    if (!tmc || !standstill) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc_read_status(tmc, &status)) {
        return false;
    }

    *standstill = (status & TMC_DRV_STATUS_STST) ? true : false;
    return true;
}

bool tsumikoro_tmc_has_error(tsumikoro_tmc_stepper_t *tmc, bool *has_error)
{
    if (!tmc || !has_error) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc_read_status(tmc, &status)) {
        return false;
    }

    /* Check for error conditions:
     * - Overtemperature (OT)
     * - Short to ground on coil A or B (S2GA, S2GB)
     * - Open load on coil A or B (OLA, OLB) */
    *has_error = (status & (TMC_DRV_STATUS_OT |
                            TMC_DRV_STATUS_S2GA |
                            TMC_DRV_STATUS_S2GB |
                            TMC_DRV_STATUS_OLA |
                            TMC_DRV_STATUS_OLB)) ? true : false;

    return true;
}

bool tsumikoro_tmc_get_stallguard(tsumikoro_tmc_stepper_t *tmc, uint16_t *result)
{
    if (!tmc || !result) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc_read_status(tmc, &status)) {
        return false;
    }

    /* Extract StallGuard result (bits [9:0]) */
    *result = (uint16_t)(status & TMC_DRV_STATUS_SG_RESULT_MASK);
    return true;
}

/* ============================================================================
 * TMC5160-Specific Motion Controller Implementation
 * ============================================================================ */

bool tsumikoro_tmc5160_set_ramp_mode(tsumikoro_tmc_stepper_t *tmc, uint8_t mode)
{
    if (!tmc || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    if (mode > TMC5160_RAMPMODE_HOLD) {
        return false;
    }

    return tsumikoro_tmc_write_register(tmc, TMC5160_REG_RAMPMODE, mode);
}

bool tsumikoro_tmc5160_set_ramp_params(tsumikoro_tmc_stepper_t *tmc,
                                       uint32_t vstart, uint32_t a1, uint32_t v1,
                                       uint32_t amax, uint32_t vmax,
                                       uint32_t dmax, uint32_t d1, uint32_t vstop)
{
    if (!tmc || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    /* Write all ramp parameters */
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VSTART, vstart)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_A1, a1)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_V1, v1)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_AMAX, amax)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VMAX, vmax)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_DMAX, dmax)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_D1, d1)) return false;
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_VSTOP, vstop)) return false;

    return true;
}

bool tsumikoro_tmc5160_move_to(tsumikoro_tmc_stepper_t *tmc, int32_t position)
{
    if (!tmc || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    /* Ensure we're in positioning mode */
    if (!tsumikoro_tmc5160_set_ramp_mode(tmc, TMC5160_RAMPMODE_POSITIONING)) {
        return false;
    }

    /* Set target position - motion starts automatically */
    return tsumikoro_tmc_write_register(tmc, TMC5160_REG_XTARGET, (uint32_t)position);
}

bool tsumikoro_tmc5160_get_position(tsumikoro_tmc_stepper_t *tmc, int32_t *position)
{
    if (!tmc || !position || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    uint32_t xactual = 0;
    if (!tsumikoro_tmc_read_register(tmc, TMC5160_REG_XACTUAL, &xactual)) {
        return false;
    }

    *position = (int32_t)xactual;
    return true;
}

bool tsumikoro_tmc5160_set_position(tsumikoro_tmc_stepper_t *tmc, int32_t position)
{
    if (!tmc || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    /* Set both XACTUAL and XTARGET to prevent unwanted movement */
    if (!tsumikoro_tmc_write_register(tmc, TMC5160_REG_XACTUAL, (uint32_t)position)) {
        return false;
    }

    return tsumikoro_tmc_write_register(tmc, TMC5160_REG_XTARGET, (uint32_t)position);
}

bool tsumikoro_tmc5160_get_velocity(tsumikoro_tmc_stepper_t *tmc, int32_t *velocity)
{
    if (!tmc || !velocity || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    uint32_t vactual = 0;
    if (!tsumikoro_tmc_read_register(tmc, TMC5160_REG_VACTUAL, &vactual)) {
        return false;
    }

    /* VACTUAL is stored as 24-bit signed value */
    if (vactual & 0x800000) {
        /* Negative velocity - sign extend */
        *velocity = (int32_t)(vactual | 0xFF000000);
    } else {
        *velocity = (int32_t)vactual;
    }

    return true;
}

bool tsumikoro_tmc5160_position_reached(tsumikoro_tmc_stepper_t *tmc, bool *reached)
{
    if (!tmc || !reached || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    uint32_t ramp_stat = 0;
    if (!tsumikoro_tmc_read_register(tmc, TMC5160_REG_RAMP_STAT, &ramp_stat)) {
        return false;
    }

    *reached = (ramp_stat & TMC5160_RAMP_STAT_POSITION_REACHED) ? true : false;
    return true;
}

bool tsumikoro_tmc5160_stop(tsumikoro_tmc_stepper_t *tmc)
{
    if (!tmc || tmc->chip_type != TMC_CHIP_TMC5160) {
        return false;
    }

    /* Read current position */
    int32_t current_pos = 0;
    if (!tsumikoro_tmc5160_get_position(tmc, &current_pos)) {
        return false;
    }

    /* Set target to current position to stop */
    return tsumikoro_tmc_write_register(tmc, TMC5160_REG_XTARGET, (uint32_t)current_pos);
}
