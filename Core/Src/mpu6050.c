/**
 * @file  mpu6050.c
 * @brief Minimal blocking I2C driver for the MPU6050 accelerometer.
 */
#include "mpu6050.h"
#include <math.h>
#include <stddef.h>

/* --- Register map (only what we touch) ---------------------------------- */
#define REG_SMPLRT_DIV      0x19u
#define REG_CONFIG          0x1Au
#define REG_ACCEL_CONFIG    0x1Cu
#define REG_ACCEL_XOUT_H    0x3Bu
#define REG_PWR_MGMT_1      0x6Bu
#define REG_PWR_MGMT_2      0x6Cu
#define REG_WHO_AM_I        0x75u

#define I2C_TIMEOUT_MS      50u

/** LSB per g at +/-2 g full scale (datasheet, section 6.2). */
#define ACCEL_LSB_PER_G     16384.0f

static I2C_HandleTypeDef *s_i2c   = NULL;
static uint16_t           s_addr  = MPU6050_ADDR_LOW;

/* ------------------------------------------------------------------------ */

static bool reg_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(s_i2c, s_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(s_i2c, s_addr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

/* ------------------------------------------------------------------------ */

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;

    s_i2c = hi2c;

    /* The AD0 pin is wired differently on different breakout boards, so try
     * both addresses instead of forcing the user to jumper it. */
    s_addr = MPU6050_ADDR_LOW;
    if (HAL_I2C_IsDeviceReady(s_i2c, s_addr, 3, I2C_TIMEOUT_MS) != HAL_OK) {
        s_addr = MPU6050_ADDR_HIGH;
        if (HAL_I2C_IsDeviceReady(s_i2c, s_addr, 3, I2C_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
    }

    if (!reg_read(REG_WHO_AM_I, &who, 1)) {
        return false;
    }
    /* Genuine MPU6050 reports 0x68. Some clones report 0x70/0x72 -- accept
     * them, they are register-compatible for what we use. */
    if (who != 0x68u && who != 0x70u && who != 0x72u) {
        return false;
    }

    /* Device reset, then wait for it to come back. */
    if (!reg_write(REG_PWR_MGMT_1, 0x80u)) return false;
    HAL_Delay(100);

    /* Wake up, clock = PLL with X-axis gyro reference. */
    if (!reg_write(REG_PWR_MGMT_1, 0x01u)) return false;
    HAL_Delay(10);

    /* Power down all three gyro axes -- saves ~3 mA, we never read them. */
    if (!reg_write(REG_PWR_MGMT_2, 0x07u)) return false;

    /* DLPF_CFG = 6 -> 5 Hz accel bandwidth. Body movement is well below
     * that, and it kills mains hum picked up by the long wearable cable. */
    if (!reg_write(REG_CONFIG, 0x06u)) return false;

    /* With DLPF enabled the base rate is 1 kHz; /(1+9) = 100 Hz. */
    if (!reg_write(REG_SMPLRT_DIV, 0x09u)) return false;

    /* AFS_SEL = 0 -> +/-2 g. */
    if (!reg_write(REG_ACCEL_CONFIG, 0x00u)) return false;

    HAL_Delay(50);
    return true;
}

bool MPU6050_ReadAccel(Accel_t *out)
{
    uint8_t buf[6];

    if (out == NULL || s_i2c == NULL) {
        return false;
    }
    if (!reg_read(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }

    int16_t rx = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    int16_t ry = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    int16_t rz = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);

    out->x = (float)rx / ACCEL_LSB_PER_G;
    out->y = (float)ry / ACCEL_LSB_PER_G;
    out->z = (float)rz / ACCEL_LSB_PER_G;
    return true;
}

float MPU6050_Magnitude(const Accel_t *a)
{
    return sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
}