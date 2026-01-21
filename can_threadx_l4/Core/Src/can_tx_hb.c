#include "can_tx_hb.h"

extern CAN_HandleTypeDef hcan1;

static CAN_TxHeaderTypeDef hb_hdr;
static uint32_t hb_mailbox;
static uint8_t hb_seq;

HAL_StatusTypeDef can_hb_init(CAN_HandleTypeDef *hcan, uint16_t stdid_hb)
{
    (void)hcan; // dùng hcan1 theo project hiện tại
    hb_hdr.StdId = stdid_hb;
    hb_hdr.IDE   = CAN_ID_STD;
    hb_hdr.RTR   = CAN_RTR_DATA;
    hb_hdr.DLC   = 8;
    hb_seq = 0;
    return HAL_OK;
}

HAL_StatusTypeDef can_hb_send(uint8_t last_scaled,
                              uint8_t flags,
                              uint8_t qfull_cnt_lsb,
                              uint8_t i2c_err_lsb)
{
    uint8_t d[8] = {0};
    d[0] = hb_seq++;
    d[1] = last_scaled;
    d[2] = flags;
    d[3] = qfull_cnt_lsb;
    d[4] = i2c_err_lsb;

    return HAL_CAN_AddTxMessage(&hcan1, &hb_hdr, d, &hb_mailbox);
}
