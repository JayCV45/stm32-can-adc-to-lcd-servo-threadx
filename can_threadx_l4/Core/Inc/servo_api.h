#ifndef SERVO_API_H
#define SERVO_API_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t tick_hz;        // timer counter clock after prescaler
    uint16_t min_us;         // default 1000
    uint16_t max_us;         // default 2000
    uint16_t last_angle_deg; // for optional change-detect
} servo_api_t;

/* Init + start PWM */
HAL_StatusTypeDef servo_api_init(servo_api_t *s,
                                 TIM_HandleTypeDef *htim,
                                 uint32_t channel,
                                 uint32_t tick_hz,
                                 uint16_t min_us,
                                 uint16_t max_us);

/* High-level setters */
void servo_api_set_angle_deg(servo_api_t *s, uint16_t angle_deg);  // 0..180
void servo_api_set_scaled_u8(servo_api_t *s, uint8_t scaled);      // 0..255 -> 0..180

/* Helpers (optional) */
uint16_t servo_api_scaled_to_angle(uint8_t scaled);
uint16_t servo_api_angle_to_pulse_us(const servo_api_t *s, uint16_t angle_deg);

#endif
