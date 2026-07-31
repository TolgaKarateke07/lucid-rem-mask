/**
 * @file  app.h
 * @brief Application layer: acquisition, state machine, cue, telemetry.
 *
 * Takes the handles CubeMX created rather than configuring anything itself,
 * so regenerating the .ioc never overwrites logic. Integration is three
 * calls in main.c -- see README.md.
 */
#ifndef APP_H
#define APP_H

#include "stm32f1xx_hal.h"

/**
 * @brief Call once after the MX_xxx_Init() functions, before the main loop.
 *
 * @param hadc         phototransistor ADC
 * @param hi2c         bus the MPU6050 sits on
 * @param htim_pwm     timer driving the cue LED
 * @param pwm_channel  its channel, e.g. TIM_CHANNEL_1
 * @param htim_tick    timer providing the 50 Hz sample interrupt
 * @param huart        telemetry UART, may be NULL
 */
void App_Init(ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              TIM_HandleTypeDef  *htim_pwm,
              uint32_t            pwm_channel,
              TIM_HandleTypeDef  *htim_tick,
              UART_HandleTypeDef *huart);

/** Run one pass. Call from while(1); returns at once if no sample is due. */
void App_Tick(void);

/** Call from HAL_TIM_PeriodElapsedCallback(). Only sets a flag. */
void App_OnSampleTimer(void);

#endif /* APP_H */
