/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "tx_api.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TX_POT_ID     0x446U
#define CAN_RX_HB_ID      0x476U

#define HB_TIMEOUT_TICKS   (TX_TIMER_TICKS_PER_SECOND * 3 / 10)  // 300ms
#define LED_BLINK_TICKS    (TX_TIMER_TICKS_PER_SECOND * 2 / 10)  // 200ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD adc_can_thread;
CHAR *adc_can_thread_stack;

/* CAN Tx stuff */
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = "NIZAR---";

// Heart beat state (global) //
static volatile uint32_t hb_rx_cnt = 0;
static volatile uint32_t hb_last_tick = 0;   // ThreadX ticks khi nhận HB gần nhất
static volatile uint8_t  hb_last_seq = 0;
static volatile uint8_t  hb_last_scaled = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID App_ThreadX_Entry(ULONG thread_input);

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */

  printf("App_ThreadX_Init: Started\r\n");

  // Allocate stack memory for the thread
    ret = tx_byte_allocate(byte_pool, (VOID **) &adc_can_thread_stack, 1024, TX_NO_WAIT);
    if (ret != TX_SUCCESS)
    {
    	printf("Stack allocation failed: %u\r\n", ret);
        return TX_POOL_ERROR;
    }
    printf("Stack allocated\r\n");

    // Create the thread
    ret = tx_thread_create(&adc_can_thread,              // Thread control block
                           "ADC_CAN_Thread",             // Name
                           App_ThreadX_Entry,            // Entry function
                           0,                            // Entry input (unused)
                           adc_can_thread_stack,         // Stack start
                           1024,                         // Stack size
                           1,                            // Priority (lower = higher priority)
                           1,                            // Preemption threshold
                           TX_NO_TIME_SLICE,             // Time slice
                           TX_AUTO_START);               // Auto start
    if (ret != TX_SUCCESS)
    {
    	printf("Thread creation failed: %u\r\n", ret);
        return TX_THREAD_ERROR;
    }
    printf("Thread created successfully\r\n");

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

VOID App_ThreadX_Entry(ULONG thread_input)
{
	(void)thread_input;

    printf("ADC_CAN_Thread started!\r\n");

    /* Start CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        printf("CAN start failed!\r\n");
    }

    /* ---- Enable notification RX  ---- */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("[F4] CAN notify RX FIFO0 failed\r\n");
    }

    /* Configure CAN Tx header */
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = CAN_TX_POT_ID;

    static ULONG last_led_tick = 0;

    while (1)
    {
        uint16_t value = 0;
        uint8_t  scaled = 0;

        /* ---- Start ADC1  ---- */
        HAL_ADC_Start(&hadc1);

        // IMPORTANT: use HAL_MAX_DELAY so this does NOT depend on HAL_GetTick()
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
        {
            value = HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            printf("ADC poll timeout (should not happen with HAL_MAX_DELAY)\r\n");
            value = 0;
        }

        HAL_ADC_Stop(&hadc1);
        /* -------------------------------------------------------- */

        scaled    = value >> 4;   // 0–4095 -> 0–255
        TxData[7] = scaled;

        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
        {
            printf("ADC Raw: %u, Scaled: %u\r\n", (unsigned int)value, scaled);
            printf("CAN Tx -> ID: 0x%03X, Data Byte: %u\r\n",
                   (unsigned int)TxHeader.StdId, TxData[7]);
        }
        else
        {
            printf("CAN Tx Failed!\r\n");
        }

        // Heartbeat LED logic
        ULONG now = tx_time_get();
        uint8_t hb_ok = 0;

        if (hb_rx_cnt > 0 && (now - hb_last_tick) <= HB_TIMEOUT_TICKS)
            hb_ok = 1;

        if (hb_ok)
        {
            if ((now - last_led_tick) >= LED_BLINK_TICKS)
            {
                last_led_tick = now;
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }
        }
        else
        {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }

        // 1 second in ThreadX ticks: your SysTick is 100 Hz (from tx_initialize_low_level.s)
        tx_thread_sleep(100);
    }
}

// Implement callback nhận heartbeat (FIFO0)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rh, d) != HAL_OK)
        return;

    if (rh.IDE != CAN_ID_STD) return;

    // Heartbeat từ L4
    if (rh.StdId == CAN_RX_HB_ID)
    {
        hb_rx_cnt++;
        hb_last_tick = tx_time_get();

        // Nếu L4 gửi DLC>=2: [seq, last_scaled,...]
        hb_last_seq = d[0];
        hb_last_scaled = d[1];
    }
}

/* USER CODE END 1 */
