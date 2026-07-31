/**
 * @file  rem_detect.h
 * @brief Eye-movement event detection and REM scoring.
 *
 * Band-pass -> adaptive threshold -> event count -> 2-of-3 window vote.
 * No HAL dependency, so it compiles and is regression-tested on a PC
 * (see tools/). Signal chain and rationale: README.md.
 */
#ifndef REM_DETECT_H
#define REM_DETECT_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    baseline;      /**< slow DC level of the reflectance signal    */
    float    ac;            /**< band-passed signal (what we threshold)     */
    float    noise;         /**< running noise floor, |AC| while quiet      */
    float    threshold;     /**< current trigger level                      */

    bool     event_now;     /**< an eye-movement event fired this sample    */
    uint16_t events_window; /**< events counted in the window in progress   */
    uint32_t events_total;  /**< since Init/Reset -- for telemetry          */

    uint8_t  rem_windows;   /**< REM-like windows in the recent history     */
    bool     rem_detected;  /**< scoring criterion currently satisfied      */
} RemDetect_t;

/** Reset every filter, counter and window. */
void RemDetect_Init(void);

/**
 * @param reflect     ambient-corrected reflectance, ADC counts
 * @param disturbed   accelerometer says this sample is untrustworthy
 * @param base_alpha  baseline adaptation rate (from Motion_BaselineAlpha)
 * @param now_ms      millisecond timestamp
 */
void RemDetect_Update(int16_t reflect, bool disturbed,
                      float base_alpha, uint32_t now_ms);

/** Current detector state. */
const RemDetect_t *RemDetect_Get(void);

/** Clear the REM score (call after a cue so the next one needs fresh evidence). */
void RemDetect_ResetScore(uint32_t now_ms);

#endif /* REM_DETECT_H */