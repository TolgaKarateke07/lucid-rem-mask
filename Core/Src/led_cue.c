/**
 * @file  led_cue.c
 * @brief Non-blocking red-LED cue sequencer (2 short, 2 long, 2 short).
 */
#include "led_cue.h"
#include "config.h"
#include <stddef.h>   /* NULL -- do not rely on the HAL header providing it */

typedef struct {
    uint16_t on_ms;    /**< how long the LED stays lit  */
    uint16_t off_ms;   /**< gap that follows it         */
} CueStep_t;

/** Sequencer states. Internal -- callers only ask LedCue_IsActive(). */
typedef enum {
    CUE_IDLE = 0,
    CUE_PULSE_ON,
    CUE_PULSE_OFF
} CueState_t;

/* 2 short, 2 long, 2 short. The trailing gap of the last step is the pause
 * before the next repetition of the whole sequence. */
static const CueStep_t k_pattern[] = {
    { CUE_SHORT_MS, CUE_GAP_MS        },
    { CUE_SHORT_MS, CUE_GROUP_GAP_MS  },
    { CUE_LONG_MS,  CUE_GAP_MS        },
    { CUE_LONG_MS,  CUE_GROUP_GAP_MS  },
    { CUE_SHORT_MS, CUE_GAP_MS        },
    { CUE_SHORT_MS, CUE_SEQUENCE_GAP_MS },
};

#define PATTERN_STEPS  (sizeof(k_pattern) / sizeof(k_pattern[0]))

static TIM_HandleTypeDef *s_tim;
static uint32_t           s_channel;

static CueState_t s_state;
static uint8_t    s_step;
static uint8_t    s_repeat;
static uint32_t   s_phase_start_ms;

/* ------------------------------------------------------------------------ */

static void pwm_write(uint8_t percent)
{
    if (s_tim == NULL) {
        return;
    }
    if (percent > 100u) {
        percent = 100u;
    }

    /* ARR is set by the timer init; deriving CCR from it keeps this correct
     * even if the PWM frequency is changed later. */
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(s_tim);
    const uint32_t ccr = ((arr + 1u) * percent) / 100u;

    __HAL_TIM_SET_COMPARE(s_tim, s_channel, ccr);
}

static void led_on(void)  { pwm_write(CUE_BRIGHTNESS_PCT); }
static void led_off(void) { pwm_write(0u); }

/* ------------------------------------------------------------------------ */

void LedCue_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    s_tim     = htim;
    s_channel = channel;
    s_state   = CUE_IDLE;
    s_step    = 0;
    s_repeat  = 0;

    HAL_TIM_PWM_Start(s_tim, s_channel);
    led_off();
}

void LedCue_Start(uint32_t now_ms)
{
    if (s_state != CUE_IDLE) {
        return;                     /* already running */
    }
    s_state          = CUE_PULSE_ON;
    s_step           = 0;
    s_repeat         = 0;
    s_phase_start_ms = now_ms;
    led_on();
}

void LedCue_Abort(void)
{
    led_off();
    s_state = CUE_IDLE;
    s_step  = 0;
    s_repeat = 0;
}

bool LedCue_IsActive(void)
{
    return s_state != CUE_IDLE;
}

void LedCue_Update(uint32_t now_ms)
{
    const uint32_t elapsed = now_ms - s_phase_start_ms;

    switch (s_state) {

    case CUE_PULSE_ON:
        if (elapsed >= k_pattern[s_step].on_ms) {
            led_off();
            s_phase_start_ms = now_ms;
            s_state = CUE_PULSE_OFF;
        }
        break;

    case CUE_PULSE_OFF:
        if (elapsed >= k_pattern[s_step].off_ms) {
            s_step++;

            if (s_step >= PATTERN_STEPS) {
                s_step = 0;
                s_repeat++;

                if (s_repeat >= CUE_REPEATS) {
                    s_state = CUE_IDLE;   /* sequence over */
                    led_off();
                    break;
                }
            }

            s_phase_start_ms = now_ms;
            s_state = CUE_PULSE_ON;
            led_on();
        }
        break;

    case CUE_IDLE:
    default:
        break;
    }
}