/**
 * @file  mpu6050.h
 * @brief Minimal blocking I2C driver, accelerometer only.
 *
 * The gyro is powered down (~3 mA saved) since we never read it.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/** 7-bit address is 0x68 (AD0 low) or 0x69 (AD0 high); HAL wants it <<1. */
#define MPU6050_ADDR_LOW    (0x68u << 1)
#define MPU6050_ADDR_HIGH   (0x69u << 1)

/** Accelerometer sample, already converted to g. */
typedef struct {
    float x;
    float y;
    float z;
} Accel_t;

/**
 * @brief  Probe, reset and configure: gyro off, 5 Hz DLPF, 100 Hz, +/-2 g.
 * @return false if no device answered at either I2C address.
 */
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read one accelerometer sample and convert to g.
 * @return true on success; on failure @p out is left untouched.
 */
bool MPU6050_ReadAccel(Accel_t *out);

/**
 * @brief  Vector magnitude in g. At rest on a table this reads ~1.0.
 */
float MPU6050_Magnitude(const Accel_t *a);

#endif /* MPU6050_H */