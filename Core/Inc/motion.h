/**
 * @file  motion.h
 * @brief Tells the REM detector when not to trust the IR signal.
 *
 * Two disturbances, handled differently: body movement only gates event
 * counting, while a posture change or mask slip also moves the DC operating
 * point and forces fast baseline re-convergence.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu6050.h"

typedef struct {
    bool  moving;        /**< movement right now, or inside the hold window  */
    bool  recalibrating; /**< baseline is in fast-adapt mode                 */
    bool  posture_change;/**< one-shot: gravity direction jumped this sample */
    float magnitude;     /**< |a| in g for this sample                       */
    float deviation;     /**< |a| - running mean, in g                       */
    float quiet_ms;      /**< how long we have been still, in ms             */
} MotionState_t;

/** Reset all internal filters. Call once at start-up. */
void Motion_Init(void);

/**
 * @brief Feed one accelerometer sample.
 * @param a       sample in g
 * @param now_ms  HAL_GetTick() value for this sample
 */
void Motion_Update(const Accel_t *a, uint32_t now_ms);

/** Current state (valid after the first Motion_Update call). */
const MotionState_t *Motion_Get(void);

/** Convenience: true while IR events must be discarded. */
bool Motion_IsDisturbed(void);

/** Convenience: baseline adaptation rate to use for this sample. */
float Motion_BaselineAlpha(void);

#endif /* MOTION_H */