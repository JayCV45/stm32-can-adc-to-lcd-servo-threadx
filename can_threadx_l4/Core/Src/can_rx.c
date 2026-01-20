#include "can_rx.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;

static TX_QUEUE can_rx_queue;
static ULONG *queue_mem;

static volatile uint32_t isr_hit_cnt = 0;
static volatile uint32_t isr_id_mismatch_cnt = 0;
static volatile uint32_t isr_q_full_cnt = 0;

uint32_t can_rx_get_isr_hit_cnt(void) { return isr_hit_cnt; }
uint32_t can_rx_get_id_mismatch_cnt(void) { return isr_id_mismatch_cnt; }
uint32_t can_rx_get_q_full_cnt(void) { return isr_q_full_cnt; }



UINT can_rx_init(TX_BYTE_POOL *pool)
{
    UINT ret;

    printf("[CAN] init: creating queue...\r\n");

    ret = tx_byte_allocate(pool, (VOID**)&queue_mem,
                               sizeof(can_pot_msg_t) * 16U, TX_NO_WAIT);
        if (ret != TX_SUCCESS) {
            printf("[CAN] queue mem alloc FAIL ret=%u\r\n", (unsigned)ret);
            return ret;
        }

        ret = tx_queue_create(&can_rx_queue, "can_rx_queue",
                                  (UINT)(sizeof(can_pot_msg_t)/sizeof(ULONG)),
                                  queue_mem, (UINT)(sizeof(can_pot_msg_t) * 16U));
            if (ret != TX_SUCCESS) {
                printf("[CAN] queue create FAIL ret=%u\r\n", (unsigned)ret);
                return ret;
            }

            printf("[CAN] queue create OK\r\n");

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 10;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterActivation = ENABLE;

    filter.FilterIdHigh     = (uint16_t)(CAN_RX_STDID << 5);
    filter.FilterIdLow      = 0;
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
    filter.FilterMaskIdLow  = 0;

#if defined(CAN_FILTERBANK_DUAL)
    filter.SlaveStartFilterBank = 14;
#endif

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        printf("[CAN] ConfigFilter FAIL\r\n");
        return TX_NOT_DONE;
    }
    printf("[CAN] ConfigFilter OK FIFO1\r\n");

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        printf("[CAN] Start FAIL\r\n");
        return TX_NOT_DONE;
    }
    printf("[CAN] Start OK\r\n");

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        printf("[CAN] Notify FAIL\r\n");
        return TX_NOT_DONE;
    }
    printf("[CAN] Notify OK (FIFO1 pending)\r\n");


    return TX_SUCCESS;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    isr_hit_cnt++;

    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hdr, data) != HAL_OK) return;

    if (hdr.IDE != CAN_ID_STD) return;
    if (hdr.StdId != CAN_RX_STDID) { isr_id_mismatch_cnt++; return; }

    can_pot_msg_t msg;
    msg.stdid = (uint16_t)hdr.StdId;
    msg.pot_raw = (uint16_t)data[7];   // 0..255

    if (tx_queue_send(&can_rx_queue, &msg, TX_NO_WAIT) != TX_SUCCESS)
        isr_q_full_cnt++;
}

UINT can_rx_receive(can_pot_msg_t *msg, ULONG timeout)
{
    return tx_queue_receive(&can_rx_queue, msg, timeout);
}
