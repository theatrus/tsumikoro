/**
 * @file tsumikoro_tmc2130.c
 * @brief TMC2130 Stepper Motor Driver Implementation
 */

#include "tsumikoro_tmc2130.h"
#include <string.h>

/* Default register values for initialization */
#define TMC2130_DEFAULT_GCONF       (0x00000000)
#define TMC2130_DEFAULT_CHOPCONF    (0x00008008)  /* TOFF=8, HSTRT=0, HEND=0 */
#define TMC2130_DEFAULT_IHOLD_IRUN  (0x00001F00)  /* IHOLD=0, IRUN=31, IHOLDDELAY=0 */
#define TMC2130_DEFAULT_TPOWERDOWN  (0x0000000A)  /* ~1 second at 12MHz */
#define TMC2130_DEFAULT_TPWMTHRS    (0x000001F4)  /* ~500 */
#define TMC2130_DEFAULT_PWMCONF     (0x000401C8)  /* Default PWM config */
#define TMC2130_DEFAULT_COOLCONF    (0x00000000)

/* Timing constants (microseconds) */
#define TMC2130_SPI_DELAY_US        1
#define TMC2130_STEP_PULSE_US       2
#define TMC2130_STEP_PAUSE_US       2

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Transfer 40-bit SPI datagram (1 address + 4 data bytes)
 *
 * @param tmc Pointer to TMC2130 handle
 * @param addr_rw Address byte with read/write flag
 * @param data_out Data to write (32-bit)
 * @param data_in Pointer to store read data (32-bit)
 */
static void tmc2130_spi_transfer_datagram(tsumikoro_tmc2130_t *tmc, uint8_t addr_rw,
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
        tmc->delay_us(TMC2130_SPI_DELAY_US);
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

bool tsumikoro_tmc2130_init(tsumikoro_tmc2130_t *tmc)
{
    if (!tmc) {
        return false;
    }

    /* Verify required callbacks are provided */
    if (!tmc->spi_begin || !tmc->spi_end || !tmc->spi_transfer) {
        return false;
    }

    /* Initialize cached register values */
    tmc->gconf = TMC2130_DEFAULT_GCONF;
    tmc->chopconf = TMC2130_DEFAULT_CHOPCONF;
    tmc->ihold_irun = TMC2130_DEFAULT_IHOLD_IRUN;
    tmc->coolconf = TMC2130_DEFAULT_COOLCONF;
    tmc->pwmconf = TMC2130_DEFAULT_PWMCONF;

    /* Configure default registers */
    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_GCONF, tmc->gconf)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_CHOPCONF, tmc->chopconf)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_IHOLD_IRUN, tmc->ihold_irun)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_TPOWERDOWN, TMC2130_DEFAULT_TPOWERDOWN)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_TPWMTHRS, TMC2130_DEFAULT_TPWMTHRS)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_PWMCONF, tmc->pwmconf)) {
        return false;
    }

    if (!tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_COOLCONF, tmc->coolconf)) {
        return false;
    }

    return true;
}

bool tsumikoro_tmc2130_read_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t *value)
{
    if (!tmc || !value || reg > 0x7F) {
        return false;
    }

    /* TMC2130 returns data from PREVIOUS read operation
     * Need to issue read twice: once to request, once to receive */

    uint32_t dummy = 0;

    /* First read: request the register */
    tmc2130_spi_transfer_datagram(tmc, reg | TMC2130_READ, 0x00000000, &dummy);

    /* Second read: receive the data (send dummy read to same register) */
    tmc2130_spi_transfer_datagram(tmc, reg | TMC2130_READ, 0x00000000, value);

    return true;
}

bool tsumikoro_tmc2130_write_register(tsumikoro_tmc2130_t *tmc, uint8_t reg, uint32_t value)
{
    if (!tmc || reg > 0x7F) {
        return false;
    }

    /* Write operation */
    tmc2130_spi_transfer_datagram(tmc, reg | TMC2130_WRITE, value, NULL);

    /* Update cached value for frequently accessed registers */
    switch (reg) {
        case TMC2130_REG_GCONF:
            tmc->gconf = value;
            break;
        case TMC2130_REG_CHOPCONF:
            tmc->chopconf = value;
            break;
        case TMC2130_REG_IHOLD_IRUN:
            tmc->ihold_irun = value;
            break;
        case TMC2130_REG_COOLCONF:
            tmc->coolconf = value;
            break;
        case TMC2130_REG_PWMCONF:
            tmc->pwmconf = value;
            break;
        default:
            break;
    }

    return true;
}

void tsumikoro_tmc2130_set_direction(tsumikoro_tmc2130_t *tmc, tsumikoro_tmc2130_direction_t dir)
{
    if (!tmc || !tmc->set_dir_pin) {
        return;
    }

    tmc->set_dir_pin(dir == TMC2130_DIR_FORWARD ? false : true);
}

void tsumikoro_tmc2130_step(tsumikoro_tmc2130_t *tmc)
{
    if (!tmc || !tmc->set_step_pin) {
        return;
    }

    /* Generate step pulse: low->high->low */
    tmc->set_step_pin(true);

    if (tmc->delay_us) {
        tmc->delay_us(TMC2130_STEP_PULSE_US);
    }

    tmc->set_step_pin(false);

    if (tmc->delay_us) {
        tmc->delay_us(TMC2130_STEP_PAUSE_US);
    }
}

void tsumikoro_tmc2130_enable(tsumikoro_tmc2130_t *tmc, bool enable)
{
    if (!tmc || !tmc->set_enable_pin) {
        return;
    }

    /* EN pin is active low: false = enabled, true = disabled */
    tmc->set_enable_pin(!enable);
}

bool tsumikoro_tmc2130_set_current(tsumikoro_tmc2130_t *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay)
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

    return tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_IHOLD_IRUN, value);
}

bool tsumikoro_tmc2130_set_microsteps(tsumikoro_tmc2130_t *tmc, uint8_t mres, bool interpolate)
{
    if (!tmc) {
        return false;
    }

    /* Clamp mres to valid range (0-8) */
    if (mres > TMC2130_MRES_FULLSTEP) {
        mres = TMC2130_MRES_FULLSTEP;
    }

    /* Modify CHOPCONF register
     * Bits [27:24] = MRES (microstep resolution)
     * Bit 28       = INTPOL (interpolation enable) */
    uint32_t chopconf = tmc->chopconf;

    /* Clear MRES and INTPOL bits */
    chopconf &= ~(TMC2130_CHOPCONF_MRES_MASK | TMC2130_CHOPCONF_INTPOL);

    /* Set new MRES value */
    chopconf |= ((uint32_t)mres << 24);

    /* Set interpolation if requested */
    if (interpolate) {
        chopconf |= TMC2130_CHOPCONF_INTPOL;
    }

    return tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_CHOPCONF, chopconf);
}

bool tsumikoro_tmc2130_set_stealth_chop(tsumikoro_tmc2130_t *tmc, bool enable)
{
    if (!tmc) {
        return false;
    }

    /* Modify GCONF register
     * Bit 2 = EN_PWM_MODE (stealthChop enable) */
    uint32_t gconf = tmc->gconf;

    if (enable) {
        gconf |= TMC2130_GCONF_EN_PWM_MODE;
    } else {
        gconf &= ~TMC2130_GCONF_EN_PWM_MODE;
    }

    return tsumikoro_tmc2130_write_register(tmc, TMC2130_REG_GCONF, gconf);
}

bool tsumikoro_tmc2130_read_status(tsumikoro_tmc2130_t *tmc, uint32_t *status)
{
    if (!tmc || !status) {
        return false;
    }

    return tsumikoro_tmc2130_read_register(tmc, TMC2130_REG_DRV_STATUS, status);
}

bool tsumikoro_tmc2130_is_standstill(tsumikoro_tmc2130_t *tmc, bool *standstill)
{
    if (!tmc || !standstill) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc2130_read_status(tmc, &status)) {
        return false;
    }

    *standstill = (status & TMC2130_DRV_STATUS_STST) ? true : false;
    return true;
}

bool tsumikoro_tmc2130_has_error(tsumikoro_tmc2130_t *tmc, bool *has_error)
{
    if (!tmc || !has_error) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc2130_read_status(tmc, &status)) {
        return false;
    }

    /* Check for error conditions:
     * - Overtemperature (OT)
     * - Short to ground on coil A or B (S2GA, S2GB)
     * - Open load on coil A or B (OLA, OLB) */
    *has_error = (status & (TMC2130_DRV_STATUS_OT |
                            TMC2130_DRV_STATUS_S2GA |
                            TMC2130_DRV_STATUS_S2GB |
                            TMC2130_DRV_STATUS_OLA |
                            TMC2130_DRV_STATUS_OLB)) ? true : false;

    return true;
}

bool tsumikoro_tmc2130_get_stallguard(tsumikoro_tmc2130_t *tmc, uint16_t *result)
{
    if (!tmc || !result) {
        return false;
    }

    uint32_t status = 0;
    if (!tsumikoro_tmc2130_read_status(tmc, &status)) {
        return false;
    }

    /* Extract StallGuard result (bits [9:0]) */
    *result = (uint16_t)(status & TMC2130_DRV_STATUS_SG_RESULT_MASK);
    return true;
}
