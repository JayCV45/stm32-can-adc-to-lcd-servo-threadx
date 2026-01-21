#include "servo_api.h"
#include <stdio.h>

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t pulse_us_to_ticks(uint32_t tick_hz, uint16_t pulse_us)
{
    // ticks = pulse_us * tick_hz / 1e6 with rounding
    return (((uint32_t)pulse_us * tick_hz) + 500000U) / 1000000U;
}

HAL_StatusTypeDef servo_api_init(servo_api_t *s,
                                 TIM_HandleTypeDef *htim,
                                 uint32_t channel,
                                 uint32_t tick_hz,
                                 uint16_t min_us,
                                 uint16_t max_us)
{
    if (!s || !htim) return HAL_ERROR;

    s->htim = htim;
    s->channel = channel;
    s->tick_hz = tick_hz;
    s->min_us = min_us;
    s->max_us = max_us;
    s->last_angle_deg = 0xFFFF;

    if (HAL_TIM_PWM_Start(s->htim, s->channel) != HAL_OK)
        return HAL_ERROR;

    // default center position
    servo_api_set_angle_deg(s, 90);
    return HAL_OK;
}

uint16_t servo_api_scaled_to_angle(uint8_t scaled)
{
    return (uint16_t)(((uint32_t)scaled * 180U) / 255U);
}

uint16_t servo_api_angle_to_pulse_us(const servo_api_t *s, uint16_t angle_deg)
{
    angle_deg = clamp_u16(angle_deg, 0, 180);

    // linear map [0..180] -> [min_us..max_us]
    uint32_t span = (uint32_t)(s->max_us - s->min_us);
    return (uint16_t)(s->min_us + (span * angle_deg) / 180U);
}

void servo_api_set_angle_deg(servo_api_t *s, uint16_t angle_deg)
{
    if (!s) return;

    angle_deg = clamp_u16(angle_deg, 0, 180);

    // Optional: avoid re-writing CCR if unchanged
    if (s->last_angle_deg == angle_deg) return;
    s->last_angle_deg = angle_deg;

    uint16_t pulse_us = servo_api_angle_to_pulse_us(s, angle_deg);
    uint32_t ticks = pulse_us_to_ticks(s->tick_hz, pulse_us);

    // DEBUG: đặt ở đây
        static uint32_t dbg_cnt = 0;
        if ((dbg_cnt++ % 50U) == 0U)   // in thưa để không spam UART
        {
            printf("[SERVO] angle=%u pulse_us=%u ticks=%lu ARR=%lu tick_hz=%lu\r\n",
                   (unsigned)angle_deg,
                   (unsigned)pulse_us,
                   (unsigned long)ticks,
                   (unsigned long)__HAL_TIM_GET_AUTORELOAD(s->htim),
                   (unsigned long)s->tick_hz);
        }

    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ticks);
}

void servo_api_set_scaled_u8(servo_api_t *s, uint8_t scaled)
{
    uint16_t angle = servo_api_scaled_to_angle(scaled);
    servo_api_set_angle_deg(s, angle);
}
