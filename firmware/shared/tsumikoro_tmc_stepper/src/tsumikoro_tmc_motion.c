/**
 * @file tsumikoro_tmc_motion.c
 * @brief Software motion controller implementation for TMC step/dir drivers
 */

#include "tsumikoro_tmc_motion.h"
#include <stdlib.h>

/* Default motion parameters */
#define DEFAULT_MAX_VELOCITY    1000    /* steps/sec */
#define DEFAULT_ACCELERATION    500     /* steps/sec^2 */
#define MIN_STEP_INTERVAL_US    100     /* Fastest step rate: 10 kHz */

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Calculate step interval from velocity
 *
 * @param velocity Velocity in steps/sec
 * @return Step interval in microseconds
 */
static uint32_t calculate_step_interval(uint32_t velocity)
{
    if (velocity == 0) {
        return 0;  /* Not moving */
    }

    /* interval_us = 1,000,000 / velocity */
    uint32_t interval = 1000000UL / velocity;

    /* Clamp to minimum interval (maximum speed) */
    if (interval < MIN_STEP_INTERVAL_US) {
        interval = MIN_STEP_INTERVAL_US;
    }

    return interval;
}

/**
 * @brief Calculate required deceleration distance
 *
 * @param current_velocity Current velocity (steps/sec)
 * @param deceleration Deceleration rate (steps/sec^2)
 * @return Distance required to decelerate to stop (steps)
 */
static int32_t calculate_decel_distance(uint32_t current_velocity, uint32_t deceleration)
{
    if (deceleration == 0) {
        return 0;
    }

    /* d = v^2 / (2 * a) */
    return (int32_t)((uint64_t)current_velocity * current_velocity / (2ULL * deceleration));
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

bool tsumikoro_tmc_motion_init(tsumikoro_tmc_motion_t *motion,
                                tsumikoro_tmc_stepper_t *tmc)
{
    if (!motion || !tmc) {
        return false;
    }

    /* Initialize structure */
    motion->tmc = tmc;
    motion->current_position = 0;
    motion->target_position = 0;
    motion->max_velocity = DEFAULT_MAX_VELOCITY;
    motion->current_velocity = 0;
    motion->acceleration = DEFAULT_ACCELERATION;
    motion->deceleration = DEFAULT_ACCELERATION;
    motion->moving = false;
    motion->enabled = false;
    motion->last_step_time_us = 0;
    motion->step_interval_us = 0;

    /* Disable motor by default */
    tsumikoro_tmc_enable(tmc, false);

    return true;
}

bool tsumikoro_tmc_motion_set_params(tsumikoro_tmc_motion_t *motion,
                                      uint32_t max_velocity,
                                      uint32_t acceleration)
{
    if (!motion) {
        return false;
    }

    /* Validate parameters */
    if (max_velocity == 0 || acceleration == 0) {
        return false;
    }

    motion->max_velocity = max_velocity;
    motion->acceleration = acceleration;
    motion->deceleration = acceleration;

    return true;
}

bool tsumikoro_tmc_motion_move_to(tsumikoro_tmc_motion_t *motion,
                                   int32_t position)
{
    if (!motion) {
        return false;
    }

    motion->target_position = position;

    if (position != motion->current_position) {
        motion->moving = true;

        /* Set direction */
        if (position > motion->current_position) {
            tsumikoro_tmc_set_direction(motion->tmc, TMC_DIR_FORWARD);
        } else {
            tsumikoro_tmc_set_direction(motion->tmc, TMC_DIR_REVERSE);
        }
    } else {
        motion->moving = false;
        motion->current_velocity = 0;
    }

    return true;
}

bool tsumikoro_tmc_motion_move_relative(tsumikoro_tmc_motion_t *motion,
                                         int32_t steps)
{
    if (!motion) {
        return false;
    }

    return tsumikoro_tmc_motion_move_to(motion, motion->current_position + steps);
}

bool tsumikoro_tmc_motion_stop(tsumikoro_tmc_motion_t *motion)
{
    if (!motion) {
        return false;
    }

    motion->target_position = motion->current_position;
    motion->moving = false;
    motion->current_velocity = 0;
    motion->step_interval_us = 0;

    return true;
}

bool tsumikoro_tmc_motion_get_position(tsumikoro_tmc_motion_t *motion,
                                        int32_t *position)
{
    if (!motion || !position) {
        return false;
    }

    *position = motion->current_position;
    return true;
}

bool tsumikoro_tmc_motion_set_position(tsumikoro_tmc_motion_t *motion,
                                        int32_t position)
{
    if (!motion) {
        return false;
    }

    motion->current_position = position;
    motion->target_position = position;
    motion->moving = false;
    motion->current_velocity = 0;

    return true;
}

bool tsumikoro_tmc_motion_is_moving(tsumikoro_tmc_motion_t *motion)
{
    if (!motion) {
        return false;
    }

    return motion->moving;
}

bool tsumikoro_tmc_motion_enable(tsumikoro_tmc_motion_t *motion,
                                  bool enable)
{
    if (!motion) {
        return false;
    }

    motion->enabled = enable;
    tsumikoro_tmc_enable(motion->tmc, enable);

    /* If disabling, stop motion */
    if (!enable) {
        tsumikoro_tmc_motion_stop(motion);
    }

    return true;
}

bool tsumikoro_tmc_motion_process(tsumikoro_tmc_motion_t *motion,
                                   uint32_t current_time_us)
{
    if (!motion || !motion->enabled || !motion->moving) {
        return false;
    }

    /* Calculate distance to target */
    int32_t distance = abs(motion->target_position - motion->current_position);

    /* Check if we've reached target */
    if (distance == 0) {
        motion->moving = false;
        motion->current_velocity = 0;
        motion->step_interval_us = 0;
        return false;
    }

    /* Calculate deceleration distance */
    int32_t decel_distance = calculate_decel_distance(motion->current_velocity,
                                                       motion->deceleration);

    /* Determine if we should accelerate, cruise, or decelerate */
    if (distance <= decel_distance && motion->current_velocity > 0) {
        /* Decelerate */
        if (motion->current_velocity > 100) {
            motion->current_velocity -= motion->deceleration / 100;  /* Rough approximation */
        } else {
            motion->current_velocity = 100;  /* Minimum velocity */
        }
    } else if (motion->current_velocity < motion->max_velocity) {
        /* Accelerate */
        motion->current_velocity += motion->acceleration / 100;  /* Rough approximation */

        if (motion->current_velocity > motion->max_velocity) {
            motion->current_velocity = motion->max_velocity;
        }
    }
    /* else: cruise at max velocity */

    /* Calculate step interval */
    motion->step_interval_us = calculate_step_interval(motion->current_velocity);

    /* Check if it's time to step */
    uint32_t elapsed = current_time_us - motion->last_step_time_us;

    if (motion->step_interval_us > 0 && elapsed >= motion->step_interval_us) {
        /* Generate step pulse */
        tsumikoro_tmc_step(motion->tmc);

        /* Update position */
        if (motion->target_position > motion->current_position) {
            motion->current_position++;
        } else {
            motion->current_position--;
        }

        /* Update last step time */
        motion->last_step_time_us = current_time_us;

        return true;  /* Step generated */
    }

    return false;  /* No step generated */
}
