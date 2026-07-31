/**
 * @file  ir_sensor.c
 * @brief IR reflectance front-end with ambient-light rejection.
 */
#include "ir_sensor.h"
#include "config.h"
#include <stddef.h>

#define ADC_TIMEOUT_MS   10u

static ADC_HandleTypeDef *s_adc = NULL;

/* ------------------------------------------------------------------------
 * Microsecond delay using the Cortex-M3 cycle counter. HAL_Delay() only has
 * 1 ms resolution, which would make each sample 2 ms longer than it needs
 * to be and waste emitter power.
 * ------------------------------------------------------------------------ */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/* ------------------------------------------------------------------------ */

static uint16_t adc_sample_avg(uint8_t n)
{
    uint32_t acc = 0;

    for (uint8_t i = 0; i < n; ++i) {
        if (HAL_ADC_Start(s_adc) != HAL_OK) {
            return 0;
        }
        if (HAL_ADC_PollForConversion(s_adc, ADC_TIMEOUT_MS) == HAL_OK) {
            acc += HAL_ADC_GetValue(s_adc);
        }
        HAL_ADC_Stop(s_adc);
    }
    return (uint16_t)(acc / n);
}

/* ------------------------------------------------------------------------ */

void IrSensor_Init(ADC_HandleTypeDef *hadc)
{
    s_adc = hadc;
    dwt_init();
    HAL_GPIO_WritePin(IR_EMITTER_PORT, IR_EMITTER_PIN, GPIO_PIN_RESET);
}

IrSample_t IrSensor_Read(void)
{
    IrSample_t s = { 0, 0, 0 };

    if (s_adc == NULL) {
        return s;
    }

    /* --- Phase 1: emitter ON ------------------------------------------- */
    HAL_GPIO_WritePin(IR_EMITTER_PORT, IR_EMITTER_PIN, GPIO_PIN_SET);
    delay_us(IR_SETTLE_US);
    s.lit = adc_sample_avg(IR_OVERSAMPLE);

    /* --- Phase 2: emitter OFF (ambient only) ---------------------------- */
    HAL_GPIO_WritePin(IR_EMITTER_PORT, IR_EMITTER_PIN, GPIO_PIN_RESET);
    delay_us(IR_SETTLE_US);
    s.dark = adc_sample_avg(IR_OVERSAMPLE);

    /* --- Ambient rejection ---------------------------------------------- */
    int32_t diff = (int32_t)s.lit - (int32_t)s.dark;
    if (diff < 0) {
        diff = 0;   /* only possible as noise around zero reflectance */
    }
    s.reflect = (int16_t)diff;

    /* Saturation check is left to the telemetry: if the 'lit' column sits
     * near 4095 there is no headroom left and the load resistor is too big. */
    return s;
}