/**
 * @file  rem_detect.c
 * @brief Eye-movement event detection and REM scoring. No HAL dependency.
 */
#include "rem_detect.h"
#include "config.h"
#include <math.h>
#include <string.h>

static RemDetect_t s_out;

static bool     s_primed;             /* baseline has been seeded          */
static uint32_t s_primed_at_ms;
static float    s_lp_fast;            /* band-pass: fast pole              */
static float    s_lp_slow;            /* band-pass: slow pole              */
static bool     s_armed;              /* Schmitt trigger ready to fire     */
static uint32_t s_last_event_ms;
static uint32_t s_window_start_ms;

/* Circular history of "was that window REM-like?" */
static bool     s_hist[REM_WINDOW_HISTORY];
static uint8_t  s_hist_idx;
static uint8_t  s_hist_len;

/** Baseline needs a few time constants before its output means anything.  */
#define WARMUP_MS   5000u

/* ------------------------------------------------------------------------ */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void history_push(bool rem_like)
{
    s_hist[s_hist_idx] = rem_like;
    s_hist_idx = (uint8_t)((s_hist_idx + 1u) % REM_WINDOW_HISTORY);
    if (s_hist_len < REM_WINDOW_HISTORY) {
        s_hist_len++;
    }
}

static uint8_t history_count(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < REM_WINDOW_HISTORY; ++i) {
        if (s_hist[i]) n++;
    }
    return n;
}

/* ------------------------------------------------------------------------ */

void RemDetect_Init(void)
{
    memset(&s_out, 0, sizeof(s_out));
    memset(s_hist, 0, sizeof(s_hist));

    s_out.noise       = EVENT_THRESH_MIN;
    s_out.threshold   = EVENT_THRESH_MIN * EVENT_K;

    s_primed          = false;
    s_primed_at_ms    = 0;
    s_lp_fast         = 0.0f;
    s_lp_slow         = 0.0f;
    s_armed           = true;
    s_last_event_ms   = 0;
    s_window_start_ms = 0;
    s_hist_idx        = 0;
    s_hist_len        = 0;
}

void RemDetect_ResetScore(uint32_t now_ms)
{
    memset(s_hist, 0, sizeof(s_hist));
    s_hist_idx        = 0;
    s_hist_len        = 0;
    s_out.rem_windows = 0;
    s_out.rem_detected = false;
    s_out.events_window = 0;
    s_window_start_ms = now_ms;
}

/* ------------------------------------------------------------------------ */

void RemDetect_Update(int16_t reflect, bool disturbed,
                      float base_alpha, uint32_t now_ms)
{
    const float x = (float)reflect;

    s_out.event_now = false;

    /* --- Seed on the very first sample --------------------------------- */
    if (!s_primed) {
        s_out.baseline    = x;
        s_out.ac          = 0.0f;
        s_lp_fast         = 0.0f;
        s_lp_slow         = 0.0f;
        s_primed          = true;
        s_primed_at_ms    = (now_ms == 0u) ? 1u : now_ms;
        s_window_start_ms = now_ms;
        return;
    }

    /* --- 1. Baseline tracking ------------------------------------------ */
    s_out.baseline += base_alpha * (x - s_out.baseline);

    /* --- 2. Band-pass: (fast LP) - (slow LP) ----------------------------
     * Fast pole kills noise above ~2.5 Hz, subtracting the slow pole kills
     * drift below ~0.16 Hz. What survives is the saccade shape.            */
    const float ac_raw = x - s_out.baseline;

    s_lp_fast += SIG_ALPHA * (ac_raw - s_lp_fast);
    s_lp_slow += BP_ALPHA  * (ac_raw - s_lp_slow);

    s_out.ac = s_lp_fast - s_lp_slow;

    const float mag = fabsf(s_out.ac);

    /* --- 3. Noise floor -------------------------------------------------
     * Only learned from quiet samples. Updating it during a movement burst
     * would inflate the floor and make the detector deaf right afterwards. */
    const bool warm = (now_ms - s_primed_at_ms) > WARMUP_MS;
    if (!disturbed && mag < s_out.threshold) {
        s_out.noise += NOISE_ALPHA * (mag - s_out.noise);
    }

    const float thr_hi = clampf(s_out.noise * EVENT_K,
                                EVENT_THRESH_MIN, EVENT_THRESH_MAX);
    const float thr_lo = clampf(s_out.noise * EVENT_K_LOW,
                                EVENT_THRESH_MIN * 0.5f, EVENT_THRESH_MAX);
    s_out.threshold = thr_hi;

    /* --- 4. Schmitt trigger with refractory period ----------------------- */
    if (!s_armed && mag < thr_lo) {
        s_armed = true;                     /* re-arm on the way back down */
    }

    if (warm && !disturbed && s_armed && mag >= thr_hi) {
        const bool refractory_ok =
            (s_out.events_total == 0u) ||
            ((now_ms - s_last_event_ms) >= EVENT_REFRACTORY_MS);

        if (refractory_ok) {
            s_out.event_now = true;
            s_out.events_window++;
            s_out.events_total++;
            s_last_event_ms = now_ms;
            s_armed = false;
        }
    }

    /* --- 5. Window scoring ---------------------------------------------- */
    if ((now_ms - s_window_start_ms) >= REM_WINDOW_MS) {
        const bool rem_like = (s_out.events_window >= REM_EVENTS_MIN);

        history_push(rem_like);
        s_out.events_window = 0;
        s_window_start_ms   = now_ms;

        s_out.rem_windows  = history_count();
        s_out.rem_detected = (s_hist_len >= REM_WINDOWS_TO_CONFIRM) &&
                             (s_out.rem_windows >= REM_WINDOWS_TO_CONFIRM);
    }
}

const RemDetect_t *RemDetect_Get(void)
{
    return &s_out;
}