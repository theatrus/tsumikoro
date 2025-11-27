/**
 * @file tsumikoro_tmc_motion.h
 * @brief Software motion controller for step/dir TMC drivers (e.g., TMC2130)
 *
 * Provides position tracking, acceleration/deceleration, and velocity control
 * for TMC drivers that use step/direction interface. TMC5160 has integrated
 * motion controller and doesn't need this.
 *
 * Copyright (c) 2025 Yann Ramin
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TSUMIKORO_TMC_MOTION_H
#define TSUMIKORO_TMC_MOTION_H

#include "tsumikoro_tmc_stepper.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motion controller state
 */
typedef struct {
    tsumikoro_tmc_stepper_t *tmc;       /**< Underlying TMC driver */

    /* Position tracking */
    int32_t current_position;            /**< Current position (steps) */
    int32_t target_position;             /**< Target position (steps) */

    /* Velocity parameters */
    uint32_t max_velocity;               /**< Maximum velocity (steps/sec) */
    uint32_t current_velocity;           /**< Current velocity (steps/sec) */

    /* Acceleration parameters */
    uint32_t acceleration;               /**< Acceleration (steps/sec^2) */
    uint32_t deceleration;               /**< Deceleration (steps/sec^2) */

    /* Motion state */
    bool moving;                         /**< True if currently moving */
    bool enabled;                        /**< True if motor is enabled */

    /* Internal timing (microseconds) */
    uint32_t last_step_time_us;          /**< Last step timestamp */
    uint32_t step_interval_us;           /**< Current step interval */
} tsumikoro_tmc_motion_t;

/**
 * @brief Initialize motion controller
 *
 * @param motion Pointer to motion controller instance
 * @param tmc Pointer to TMC driver
 * @return true on success, false on error
 */
bool tsumikoro_tmc_motion_init(tsumikoro_tmc_motion_t *motion,
                                tsumikoro_tmc_stepper_t *tmc);

/**
 * @brief Set motion parameters
 *
 * @param motion Pointer to motion controller
 * @param max_velocity Maximum velocity (steps/sec)
 * @param acceleration Acceleration (steps/sec^2)
 * @return true on success
 */
bool tsumikoro_tmc_motion_set_params(tsumikoro_tmc_motion_t *motion,
                                      uint32_t max_velocity,
                                      uint32_t acceleration);

/**
 * @brief Move to absolute position
 *
 * @param motion Pointer to motion controller
 * @param position Target position (steps)
 * @return true on success
 */
bool tsumikoro_tmc_motion_move_to(tsumikoro_tmc_motion_t *motion,
                                   int32_t position);

/**
 * @brief Move relative to current position
 *
 * @param motion Pointer to motion controller
 * @param steps Steps to move (positive or negative)
 * @return true on success
 */
bool tsumikoro_tmc_motion_move_relative(tsumikoro_tmc_motion_t *motion,
                                         int32_t steps);

/**
 * @brief Stop motion immediately
 *
 * Sets target position to current position.
 *
 * @param motion Pointer to motion controller
 * @return true on success
 */
bool tsumikoro_tmc_motion_stop(tsumikoro_tmc_motion_t *motion);

/**
 * @brief Get current position
 *
 * @param motion Pointer to motion controller
 * @param position Pointer to store current position
 * @return true on success
 */
bool tsumikoro_tmc_motion_get_position(tsumikoro_tmc_motion_t *motion,
                                        int32_t *position);

/**
 * @brief Set current position (homing)
 *
 * @param motion Pointer to motion controller
 * @param position New current position value
 * @return true on success
 */
bool tsumikoro_tmc_motion_set_position(tsumikoro_tmc_motion_t *motion,
                                        int32_t position);

/**
 * @brief Check if motor is moving
 *
 * @param motion Pointer to motion controller
 * @return true if moving, false if stopped
 */
bool tsumikoro_tmc_motion_is_moving(tsumikoro_tmc_motion_t *motion);

/**
 * @brief Enable/disable motor
 *
 * @param motion Pointer to motion controller
 * @param enable true to enable, false to disable
 * @return true on success
 */
bool tsumikoro_tmc_motion_enable(tsumikoro_tmc_motion_t *motion,
                                  bool enable);

/**
 * @brief Process motion controller (call periodically)
 *
 * This function should be called frequently (e.g., every 100us to 1ms)
 * to generate step pulses and update motion state. Pass current time
 * in microseconds.
 *
 * @param motion Pointer to motion controller
 * @param current_time_us Current time in microseconds
 * @return true if step was generated
 */
bool tsumikoro_tmc_motion_process(tsumikoro_tmc_motion_t *motion,
                                   uint32_t current_time_us);

#ifdef __cplusplus
}
#endif

#endif /* TSUMIKORO_TMC_MOTION_H */
