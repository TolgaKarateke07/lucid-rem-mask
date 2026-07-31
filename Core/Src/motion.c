/**
 * @file  motion.c
 * @brief Movement / artefact detector built on the accelerometer.
 */
#include "motion.h"
#include "config.h"
#include <math.h>

static MotionState_t s_state;

static float    s_mag_mean;          /* running mean of |a|                  */
static Accel_t  s_grav;              /* low-passed gravity vector            */
static bool     s_primed;            /* filters have seen their first sample */
static uint32_t s_last_motion_ms;
static uint32_t s_recal_until_ms;
static uint32_t s_quiet_since_ms;

/* ------------------------------------------------------------------------ */

void Motion_Init(void)
{
    s_state.moving         = false;
    s_state.recalibrating  = false;
    s_state.posture_change = false;
    s_state.magnitude      = 0.0f;
    s_state.deviation      = 0.0f;
    s_state.quiet_ms       = 0.0f;

    s_mag_mean       = 1.0f;      /* 1 g is the correct prior at rest */
    s_grav.x         = 0.0f;
    s_grav.y         = 0.0f;
    s_grav.z         = 1.0f;
    s_primed         = false;
    s_last_motion_ms = 0;
    s_recal_until_ms = 0;
    s_quiet_since_ms = 0;
}

void Motion_Update(const Accel_t *a, uint32_t now_ms)
{
    const float mag = MPU6050_Magnitude(a);

    /* First sample: seed the filters instead of letting them converge from
     * an arbitrary state, which would look like a huge fake movement. */
    if (!s_primed) {
        s_mag_mean       = mag;
        s_grav           = *a;
        s_primed         = true;
        s_quiet_since_ms = now_ms;
    }

    /* --- 1. Body movement: deviation from the running mean --------------- */
    const float dev = fabsf(mag - s_mag_mean);
    s_mag_mean += ACCEL_ALPHA * (mag - s_mag_mean);

    /* --- 2. Posture change: how far the gravity vector moved ------------- */
    const float dx = a->x - s_grav.x;
    const float dy = a->y - s_grav.y;
    const float dz = a->z - s_grav.z;
    const float tilt = sqrtf(dx * dx + dy * dy + dz * dz);

    s_grav.x += ACCEL_ALPHA * dx;
    s_grav.y += ACCEL_ALPHA * dy;
    s_grav.z += ACCEL_ALPHA * dz;

    const bool posture = (tilt > MOTION_TILT_THRESH);
    const bool active  = (dev > MOTION_G_THRESH) || posture;

    if (active) {
        s_last_motion_ms = now_ms;
        s_quiet_since_ms = now_ms;

        /* A posture change invalidates the IR operating point, so schedule
         * fast baseline re-convergence for a while after it settles. */
        if (posture) {
            s_recal_until_ms = now_ms + MOTION_HOLD_MS + RECAL_MS;
        }
    }

    /* Hold the "moving" flag after the last active sample: the mask keeps
     * settling mechanically for a second or two after the body stops. */
    const bool in_hold = (now_ms - s_last_motion_ms) < MOTION_HOLD_MS;

    s_state.moving         = active || in_hold;
    s_state.recalibrating  = (int32_t)(s_recal_until_ms - now_ms) > 0;
    s_state.posture_change = posture;
    s_state.magnitude      = mag;
    s_state.deviation      = dev;
    s_state.quiet_ms       = s_state.moving
                           ? 0.0f
                           : (float)(now_ms - s_quiet_since_ms);
}

const MotionState_t *Motion_Get(void)
{
    return &s_state;
}

bool Motion_IsDisturbed(void)
{
    return s_state.moving;
}

float Motion_BaselineAlpha(void)
{
    /* Fast adaptation while disturbed or during the post-disturbance
     * recalibration window; slow drift-tracking otherwise. */
    return (s_state.moving || s_state.recalibrating)
         ? BASE_ALPHA_FAST
         : BASE_ALPHA_SLOW;
}