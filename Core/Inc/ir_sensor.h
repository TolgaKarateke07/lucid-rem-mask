/**
 * @file  ir_sensor.h
 * @brief IR reflectance front-end with ambient-light rejection.
 *
 * Each sample is a pair -- emitter ON then OFF -- and the difference cancels
 * any light present in both. Same principle as an IR remote receiver.
 */
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/** One conditioned IR sample. */
typedef struct {
    uint16_t lit;      /**< averaged ADC counts, emitter ON               */
    uint16_t dark;     /**< averaged ADC counts, emitter OFF (ambient)    */
    int16_t  reflect;  /**< lit - dark, clamped at 0: the useful signal   */
} IrSample_t;

/**
 * @brief Bind the driver to an initialised ADC handle and park the emitter off.
 */
void IrSensor_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief Perform one lit/dark measurement pair.
 *
 * Blocking, takes roughly 2 * (IR_SETTLE_US + IR_OVERSAMPLE * t_conv), i.e.
 * well under 1 ms -- comfortably inside a 20 ms sample slot.
 */
IrSample_t IrSensor_Read(void);

#endif /* IR_SENSOR_H */