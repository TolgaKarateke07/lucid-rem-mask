/**
 * @file  app.c
 * @brief Application layer: acquisition, state machine, cue, telemetry.
 *
 * State machine
 * -------------
 *   BOOT -> CALIBRATE -> WAIT_SLEEP -> MONITOR -> CUE -> REFRACTORY
 *             15 s        stillness      REM?    pattern   15 min
 *                         >= 10 min                           |
 *                                          ^                  |
 *                                          +------------------+
 */
#include "app.h"
#include "config.h"
#include "mpu6050.h"
#include "ir_sensor.h"
#include "motion.h"
#include "rem_detect.h"
#include "led_cue.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------ */

typedef enum {
    APP_BOOT = 0,
    APP_CALIBRATE,
    APP_WAIT_SLEEP,
    APP_MONITOR,
    APP_CUE,
    APP_REFRACTORY
} AppState_t;

static UART_HandleTypeDef *s_uart;

static volatile bool s_sample_due = false;   /* set by the timer ISR */

static AppState_t s_state          = APP_BOOT;
static uint32_t   s_state_entry_ms = 0;
static uint32_t   s_sleep_onset_ms = 0;
static uint32_t   s_last_cue_ms    = 0;
static uint32_t   s_cue_count      = 0;
static bool       s_accel_ok       = false;  /* false -> IR only, degraded */

/** The baseline is seeded on the first sample; this window just lets the
 *  noise-floor estimate settle before thresholds are trusted.             */
#define CALIBRATE_MS   15000u

static void status_led_task(uint32_t now_ms);
static void telemetry_task(uint32_t now_ms, const IrSample_t *ir);

static void set_state(AppState_t next, uint32_t now_ms)
{
    s_state          = next;
    s_state_entry_ms = now_ms;
}

/* ------------------------------------------------------------------------ */

void App_Init(ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              TIM_HandleTypeDef  *htim_pwm,
              uint32_t            pwm_channel,
              TIM_HandleTypeDef  *htim_tick,
              UART_HandleTypeDef *huart)
{
    s_uart = huart;

    IrSensor_Init(hadc);
    LedCue_Init(htim_pwm, pwm_channel);
    Motion_Init();
    RemDetect_Init();

    s_accel_ok = MPU6050_Init(hi2c);

#if TELEMETRY_ENABLE
    if (s_uart != NULL) {
        const char *banner =
            "\r\n# lucid-rem-mask  STM32F103C8T6\r\n"
            "# t_ms,state,lit,dark,reflect,baseline,ac,thr,ev_win,ev_tot,"
            "mag_mg,dev_mg,quiet_s,motion,rem\r\n";
        HAL_UART_Transmit(s_uart, (uint8_t *)banner,
                          (uint16_t)strlen(banner), 100);

        if (!s_accel_ok) {
            const char *warn = "# WARNING: MPU6050 not found -> running "
                               "without motion rejection\r\n";
            HAL_UART_Transmit(s_uart, (uint8_t *)warn,
                              (uint16_t)strlen(warn), 100);
        }
    }
#endif

    /* Start the 50 Hz sample tick last, so nothing fires before we are ready. */
    HAL_TIM_Base_Start_IT(htim_tick);

    set_state(APP_CALIBRATE, HAL_GetTick());
}

void App_OnSampleTimer(void)
{
    s_sample_due = true;
}

/* ------------------------------------------------------------------------ */

void App_Tick(void)
{
    const uint32_t now = HAL_GetTick();

    /* Ticked every loop pass, not once per sample: the cue's timing
     * resolution is then not limited by the 20 ms sample slot. */
    LedCue_Update(now);
    status_led_task(now);

    if (!s_sample_due) {
        return;
    }
    s_sample_due = false;

    /* --- Acquire -------------------------------------------------------- */
    const IrSample_t ir = IrSensor_Read();

    bool disturbed = false;
    if (s_accel_ok) {
        Accel_t a;
        if (MPU6050_ReadAccel(&a)) {
            Motion_Update(&a, now);
            disturbed = Motion_IsDisturbed();
        } else {
            /* Bus glitch: assume disturbed rather than trust a stale value.
             * Worst case we discard one sample out of fifty. */
            disturbed = true;
        }
    }

    /* --- Process -------------------------------------------------------- */
    const float alpha = s_accel_ok ? Motion_BaselineAlpha() : BASE_ALPHA_SLOW;
    RemDetect_Update(ir.reflect, disturbed, alpha, now);

    const RemDetect_t   *det = RemDetect_Get();
    const MotionState_t *mot = Motion_Get();

    /* --- State machine -------------------------------------------------- */
    switch (s_state) {

    case APP_CALIBRATE:
        if ((now - s_state_entry_ms) >= CALIBRATE_MS) {
            set_state(APP_WAIT_SLEEP, now);
        }
        break;

    case APP_WAIT_SLEEP:
        /* Sleep onset is inferred from sustained stillness. Without EEG this
         * is the best proxy available, and it only has to be approximate --
         * its job is to start the 90 minute latency clock. */
        if (!s_accel_ok ||
            (!mot->moving && mot->quiet_ms >= (float)SLEEP_ONSET_QUIET_MS)) {
            s_sleep_onset_ms = now;
            set_state(APP_MONITOR, now);
        }
        break;

    case APP_MONITOR: {
        const bool latency_ok    = (now - s_sleep_onset_ms) >= SLEEP_LATENCY_MS;
        const bool refractory_ok = (s_cue_count == 0u) ||
                                   ((now - s_last_cue_ms) >= CUE_REFRACTORY_MS);

        if (det->rem_detected && latency_ok && refractory_ok && !disturbed) {
            LedCue_Start(now);
            set_state(APP_CUE, now);
        }
        break;
    }

    case APP_CUE:
#if CUE_ABORT_ON_MOTION
        /* A big movement mid-cue almost always means we woke them. Stopping
         * instantly makes the awakening shorter. */
        if (s_accel_ok && mot->posture_change) {
            LedCue_Abort();
            s_last_cue_ms = now;
            s_cue_count++;
            RemDetect_ResetScore(now);
            set_state(APP_REFRACTORY, now);
            break;
        }
#endif
        if (!LedCue_IsActive()) {
            s_last_cue_ms = now;
            s_cue_count++;
            RemDetect_ResetScore(now);
            set_state(APP_REFRACTORY, now);
        }
        break;

    case APP_REFRACTORY:
        if ((now - s_last_cue_ms) >= CUE_REFRACTORY_MS) {
            set_state(APP_MONITOR, now);
        }
        break;

    case APP_BOOT:
    default:
        set_state(APP_CALIBRATE, now);
        break;
    }

    telemetry_task(now, &ir);
}

/* ------------------------------------------------------------------------ */

/**
 * @brief Status LED blink patterns.
 *
 * With the mask on your face this is the only feedback available, so every
 * state gets a rate you can tell apart. PC13 is active LOW on the Blue Pill.
 */
static void status_led_task(uint32_t now_ms)
{
    uint32_t period = 0;
    uint32_t duty   = 0;

    switch (s_state) {
    case APP_CALIBRATE:  period = 200u;  duty = 100u; break;  /* fast blink  */
    case APP_WAIT_SLEEP: period = 2000u; duty = 100u; break;  /* short chirp */
    case APP_MONITOR:    period = 4000u; duty = 50u;  break;  /* rare blip   */
    case APP_CUE:        period = 0u;    duty = 0u;   break;  /* stay dark   */
    case APP_REFRACTORY: period = 4000u; duty = 300u; break;
    default:             period = 100u;  duty = 50u;  break;
    }

    bool on = false;
    if (period != 0u) {
        on = ((now_ms % period) < duty);
    }
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief Emit one CSV row. Integers only -- newlib's %f costs several kB of
 *        flash; the Python tool rescales the fixed-point values on the host.
 */
static void telemetry_task(uint32_t now_ms, const IrSample_t *ir)
{
#if TELEMETRY_ENABLE
    if (s_uart == NULL) {
        return;
    }

    const RemDetect_t   *d = RemDetect_Get();
    const MotionState_t *m = Motion_Get();

    char line[160];
    const int n = snprintf(line, sizeof(line),
        "%lu,%u,%u,%u,%d,%ld,%ld,%ld,%u,%lu,%d,%d,%d,%u,%u\r\n",
        (unsigned long)now_ms,
        (unsigned)s_state,
        (unsigned)ir->lit,
        (unsigned)ir->dark,
        (int)ir->reflect,
        (long)d->baseline,
        (long)d->ac,
        (long)d->threshold,
        (unsigned)d->events_window,
        (unsigned long)d->events_total,
        (int)(m->magnitude * 1000.0f),
        (int)(m->deviation * 1000.0f),
        (int)(m->quiet_ms / 1000.0f),
        (unsigned)(m->moving ? 1u : 0u),
        (unsigned)(d->rem_detected ? 1u : 0u));

    if (n > 0) {
        HAL_UART_Transmit(s_uart, (uint8_t *)line, (uint16_t)n, 50);
    }
#else
    (void)now_ms; (void)ir;
#endif
}