/**
 * @file  led_cue.h
 * @brief Non-blocking cue sequencer: 2 short, 2 long, 2 short.
 *
 * State machine rather than HAL_Delay(), so sampling continues during the
 * 23 s cue. PWM rather than GPIO, because brightness is the parameter that
 * decides whether the cue reaches the dream or wakes the sleeper.
 */
#ifndef LED_CUE_H
#define LED_CUE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/** Bind to the PWM timer and force the LED off. */
void LedCue_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/** Begin the sequence. Ignored if one is already running. */
void LedCue_Start(uint32_t now_ms);

/** Stop immediately and blank the LED (used when the user moves/wakes). */
void LedCue_Abort(void);

/** Tick the sequencer. Call every main-loop iteration. */
void LedCue_Update(uint32_t now_ms);

/** True while a sequence is in progress. */
bool LedCue_IsActive(void);

#endif /* LED_CUE_H */