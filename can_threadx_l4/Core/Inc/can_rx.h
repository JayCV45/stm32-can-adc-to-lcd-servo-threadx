#ifndef CAN_RX_H
#define CAN_RX_H

#include "stm32l4xx_hal.h"
#include "tx_api.h"

#define CAN_RX_STDID   (0x446U)

uint32_t can_rx_get_isr_hit_cnt(void);
uint32_t can_rx_get_id_mismatch_cnt(void);
uint32_t can_rx_get_q_full_cnt(void);


/* Message format pushed to queue */
typedef struct
{
    uint16_t stdid;
    uint16_t pot_raw;		// pot_scaled now, scaled value
} can_pot_msg_t;

/* API */
UINT  can_rx_init(TX_BYTE_POOL *pool);
UINT  can_rx_receive(can_pot_msg_t *msg, ULONG timeout);

/* ISR hook */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
