/**
 * @file  test_rem_detect.c
 * @brief Host-side regression test for the REM detector.
 *
 * The detector's ground truth happens while you are asleep, which makes it
 * almost impossible to debug on the target. So the algorithm is written
 * without any HAL dependency and exercised here against a synthetic night
 * with known phases. Any tuning change can be checked in seconds instead of
 * costing a night of sleep.
 *
 * Build & run:
 *     cc -I../Core/Inc -o test_rem_detect test_rem_detect.c \
 *        ../Core/Src/rem_detect.c -lm && ./test_rem_detect
 *
 * Synthetic night (50 Hz):
 *   phase A   0-180 s   quiet NREM   -> expect NO REM
 *   phase B 180-360 s   REM bursts   -> expect REM detected
 *   phase C 360-420 s   movement     -> expect NO new events (rejected)
 *   phase D 420-600 s   quiet NREM   -> expect REM flag to clear
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "rem_detect.h"

#define FS            SAMPLE_RATE_HZ
#define DT_MS         (1000u / FS)

#define PHASE_A_END_S 180
#define PHASE_B_END_S 360
#define PHASE_C_END_S 420
#define PHASE_D_END_S 600

/* Deterministic PRNG so the test result is reproducible. */
static uint32_t rng_state = 0xC0FFEEu;
static float urand(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)((rng_state >> 8) & 0xFFFFFFu) / (float)0x1000000u;
}
/* Box-Muller, good enough for sensor-noise simulation. */
static float gauss(float sigma)
{
    float u1 = urand(); if (u1 < 1e-6f) u1 = 1e-6f;
    float u2 = urand();
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

int main(void)
{
    RemDetect_Init();

    const int    total    = PHASE_D_END_S * FS;
    const float  dc       = 2000.0f;   /* nominal reflectance, ADC counts */
    const float  noise_sd = 4.0f;

    /* Saccade state */
    int   sacc_left   = 0;
    float sacc_amp    = 0.0f;
    int   next_sacc   = 0;

    unsigned ev_a = 0, ev_b = 0, ev_c = 0, ev_d = 0;
    bool rem_in_a = false, rem_in_b = false, rem_at_end = false;
    unsigned prev_total = 0;

    for (int i = 0; i < total; ++i) {
        const uint32_t t_ms = (uint32_t)i * DT_MS;
        const int      t_s  = i / (int)FS;

        /* --- Build the synthetic reflectance signal --------------------- */
        float x = dc;

        /* Slow thermal/skin drift: +/-40 counts over the whole night. */
        x += 40.0f * sinf(6.2831853f * (float)i / (float)(200 * FS));

        bool disturbed = false;

        if (t_s >= PHASE_B_END_S && t_s < PHASE_C_END_S) {
            /* Movement: big low-frequency excursion + flagged as disturbed */
            x += 250.0f * sinf(6.2831853f * (float)i / (float)(2 * FS));
            x += gauss(30.0f);
            disturbed = true;
        } else if (t_s >= PHASE_A_END_S && t_s < PHASE_B_END_S) {
            /* REM: saccade bursts roughly every 2-4 s */
            if (sacc_left == 0 && i >= next_sacc) {
                sacc_left = (int)(0.30f * FS);          /* ~300 ms deflection */
                sacc_amp  = (urand() > 0.5f ? 1.0f : -1.0f) *
                            (50.0f + 30.0f * urand());
                next_sacc = i + (int)((2.0f + 2.0f * urand()) * FS);
            }
        }

        if (sacc_left > 0) {
            const float phase = (float)sacc_left / (0.30f * FS);
            x += sacc_amp * sinf(3.14159265f * phase);
            sacc_left--;
        }

        x += gauss(noise_sd);

        /* --- Feed the detector ------------------------------------------ */
        RemDetect_Update((int16_t)lrintf(x), disturbed,
                         disturbed ? BASE_ALPHA_FAST : BASE_ALPHA_SLOW, t_ms);

        const RemDetect_t *d = RemDetect_Get();

        const unsigned delta = d->events_total - prev_total;
        prev_total = d->events_total;

        if      (t_s < PHASE_A_END_S) { ev_a += delta; rem_in_a |= d->rem_detected; }
        else if (t_s < PHASE_B_END_S) { ev_b += delta; rem_in_b |= d->rem_detected; }
        else if (t_s < PHASE_C_END_S) { ev_c += delta; }
        else                          { ev_d += delta; rem_at_end = d->rem_detected; }
    }

    /* --- Report --------------------------------------------------------- */
    const RemDetect_t *d = RemDetect_Get();
    printf("phase A (quiet NREM, 180 s) : %3u events, REM flagged = %s\n",
           ev_a, rem_in_a ? "YES" : "no");
    printf("phase B (REM, 180 s)        : %3u events, REM flagged = %s\n",
           ev_b, rem_in_b ? "YES" : "no");
    printf("phase C (movement, 60 s)    : %3u events (should be 0)\n", ev_c);
    printf("phase D (quiet NREM, 180 s) : %3u events, REM flag at end = %s\n",
           ev_d, rem_at_end ? "YES" : "no");
    printf("final noise floor = %.2f, threshold = %.2f\n",
           d->noise, d->threshold);

    /* --- Assertions ----------------------------------------------------- */
    int fail = 0;

    if (rem_in_a) {
        printf("FAIL: REM declared during quiet NREM (false positive)\n");
        fail = 1;
    }
    if (!rem_in_b) {
        printf("FAIL: REM not detected during the REM phase\n");
        fail = 1;
    }
    if (ev_c != 0u) {
        printf("FAIL: %u events counted during flagged movement\n", ev_c);
        fail = 1;
    }
    if (rem_at_end) {
        printf("FAIL: REM flag never cleared after the REM phase ended\n");
        fail = 1;
    }
    if (ev_a > 10u) {
        printf("FAIL: too many events in quiet NREM (%u) -- threshold too low\n",
               ev_a);
        fail = 1;
    }

    printf(fail ? "\nRESULT: FAIL\n" : "\nRESULT: PASS\n");
    return fail;
}
