/**
 * @file    config.h
 * @brief   Compile-time configuration: pin map, timing and detection tuning.
 * @target  STM32F103C8T6 @ 72 MHz (HSE 8 MHz + PLL x9)
 *
 * Every "magic number" in this firmware lives here. Tune the detector by
 * editing this file only -- the algorithm sources should not need changes.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ==========================================================================
 * 1. PINS WE DRIVE OURSELVES
 * ==========================================================================
 * Only pins this code toggles directly. Everything else belongs to the .ioc;
 * duplicating it here would let the two drift apart. Pin map: README.md.
 */

/* PA1 : IR emitter drive. Do NOT sink the LED current from the MCU pin --
 * use an NPN (e.g. 2N3904) or small MOSFET; PA1 drives the base/gate.       */
#define IR_EMITTER_PORT         GPIOA
#define IR_EMITTER_PIN          GPIO_PIN_1

/* PC13 : on-board Blue Pill status LED, ACTIVE LOW.                         */
#define STATUS_LED_PORT         GPIOC
#define STATUS_LED_PIN          GPIO_PIN_13

/* ==========================================================================
 * 2. SAMPLING
 * ========================================================================== */

/** Must match TIM2 in the .ioc (PSC 7199, ARR 199 -> 50 Hz). Every filter
 *  constant below assumes this rate; change it and all time constants move. */
#define SAMPLE_RATE_HZ          50u

/** Emitter rise + phototransistor settling before the "lit" ADC sample.     */
#define IR_SETTLE_US            250u

/** Number of ADC conversions averaged per phase (lit / dark). Cheap noise
 *  reduction; 4 costs ~60 us at 55.5-cycle sampling time.                    */
#define IR_OVERSAMPLE           4u

/* ==========================================================================
 * 3. IR SIGNAL CONDITIONING
 * ==========================================================================
 * The chain is:  raw -> slow EMA baseline -> AC = raw-baseline -> fast EMA,
 * which is effectively a 0.05-5 Hz band-pass built from two one-pole filters.
 */

/** Slow baseline adaptation (normal operation). At 50 Hz, alpha=0.0015 gives
 *  a time constant of ~13 s: slow enough to pass a 0.3 s saccade untouched,
 *  fast enough to track skin/temperature drift.                              */
#define BASE_ALPHA_SLOW         0.0015f

/** Fast baseline adaptation, used for RECAL_MS after the mask is disturbed
 *  so the DC level re-converges in ~0.3 s instead of 13 s.                   */
#define BASE_ALPHA_FAST         0.0600f

/** Output smoothing of the AC component (one-pole low-pass, ~2.5 Hz).        */
#define SIG_ALPHA               0.3000f

/** High-pass corner of the detection band-pass (tau = 1 s -> ~0.16 Hz).
 *  The baseline alone leaves ~40 % of slow drift behind, which inflates the
 *  noise floor; this second pole removes it. Measurements in README.md.     */
#define BP_ALPHA                0.0200f

/** Adaptation rate of the running noise floor (mean |AC| while quiet).       */
#define NOISE_ALPHA             0.0050f

/* ==========================================================================
 * 4. EYE-MOVEMENT EVENT DETECTION
 * ========================================================================== */

/** Schmitt trigger high threshold = EVENT_K x noise floor, clamped.
 *  Raise EVENT_K if you get events while awake and still; lower it if a
 *  known REM period produces no events.                                      */
#define EVENT_K                 3.5f
#define EVENT_THRESH_MIN        8.0f      /* ADC LSB, absolute floor          */
#define EVENT_THRESH_MAX        600.0f    /* ADC LSB, absolute ceiling        */

/** Hysteresis: signal must fall back below K_LOW x noise before the detector
 *  re-arms. Prevents one saccade being counted several times.                */
#define EVENT_K_LOW             1.5f

/** Minimum spacing between two counted events.                              */
#define EVENT_REFRACTORY_MS     150u

/* ==========================================================================
 * 5. MOTION / ARTEFACT REJECTION  (MPU6050)
 * ========================================================================== */

/** |a| deviation from the running mean that counts as body movement, in g.   */
#define MOTION_G_THRESH         0.060f

/** Change in the gravity direction that counts as a posture change (rolling
 *  over). Compared against the norm of the gravity-vector delta, in g.       */
#define MOTION_TILT_THRESH      0.150f

/** After the last motion sample, keep rejecting IR events for this long --
 *  the phototransistor keeps ringing after the mask stops moving.            */
#define MOTION_HOLD_MS          3000u

/** Duration of fast-baseline re-convergence after a disturbance ends.        */
#define RECAL_MS                8000u

/** Accelerometer running-mean adaptation rate.                               */
#define ACCEL_ALPHA             0.0100f

/* ==========================================================================
 * 6. REM SCORING
 * ========================================================================== */

/** Sliding window over which eye-movement events are counted.               */
#define REM_WINDOW_MS           30000u

/** Events within one window to call that window "REM-like".                 */
#define REM_EVENTS_MIN          5u

/** Of the last REM_WINDOW_HISTORY windows, how many must be REM-like before
 *  we declare REM. 2-of-3 rejects isolated bursts (e.g. a swallow).          */
#define REM_WINDOW_HISTORY      3u
#define REM_WINDOWS_TO_CONFIRM  2u

/** No cue before this much time has passed since sleep onset. The first REM
 *  period normally starts ~90 min after falling asleep.                      */
#define SLEEP_LATENCY_MS        (90UL * 60UL * 1000UL)

/** Minimum quiet time (no motion) before we accept that the user is asleep.  */
#define SLEEP_ONSET_QUIET_MS    (10UL * 60UL * 1000UL)

/** After a cue, do not cue again for this long.                             */
#define CUE_REFRACTORY_MS       (15UL * 60UL * 1000UL)

/* ==========================================================================
 * 7. CUE PATTERN  --  2 short, 2 long, 2 short
 * ========================================================================== */

/** PWM duty of the red LED, 0-100 %. Start LOW (20-35 %). A cue that wakes
 *  you up is a failed cue.                                                   */
#define CUE_BRIGHTNESS_PCT      30u

#define CUE_SHORT_MS            200u   /* short flash on-time                */
#define CUE_LONG_MS             700u   /* long flash on-time                 */
#define CUE_GAP_MS              200u   /* gap inside a pair                  */
#define CUE_GROUP_GAP_MS        500u   /* gap between short/long groups      */
#define CUE_SEQUENCE_GAP_MS     4000u  /* gap between whole repetitions      */

/** How many times the 2-short / 2-long / 2-short sequence repeats.          */
#define CUE_REPEATS             3u

/** Abort the cue immediately if the user moves (they probably woke up).     */
#define CUE_ABORT_ON_MOTION     1

/* ==========================================================================
 * 8. DEBUG
 * ========================================================================== */

/** 1 = stream CSV telemetry on USART1 (115200 8N1) at the sample rate.
 *  Set to 0 for a battery run once tuning is finished.                      */
#define TELEMETRY_ENABLE        1

#endif /* CONFIG_H */
