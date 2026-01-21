#ifndef CAN_TX_HB_H
#define CAN_TX_HB_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef can_hb_init(CAN_HandleTypeDef *hcan,
                              uint16_t stdid_hb);

HAL_StatusTypeDef can_hb_send(uint8_t last_scaled,
                              uint8_t flags,
                              uint8_t qfull_cnt_lsb,
                              uint8_t i2c_err_lsb);

#endif
