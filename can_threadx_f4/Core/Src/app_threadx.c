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
    printf("ADC_CAN_Thread started!\r\n");

    /* Start CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        printf("CAN start failed!\r\n");
    }

    /* Configure CAN Tx header */
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x446;

    while (1)
    {
        uint16_t value;
        uint8_t  scaled;

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

        // 1 second in ThreadX ticks: your SysTick is 100 Hz (from tx_initialize_low_level.s)
        tx_thread_sleep(100);
    }
}

/* USER CODE END 1 */
