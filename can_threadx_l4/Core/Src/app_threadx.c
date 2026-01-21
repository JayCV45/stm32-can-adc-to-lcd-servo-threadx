/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include <string.h>
#include "lcd_i2c.h"
#include "can_rx.h"
#include "servo_api.h"
#include "can_tx_hb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim2;

extern CAN_HandleTypeDef hcan1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HB_PERIOD_MS 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD i2c_can_thread;
CHAR *i2c_can_thread_stack;

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

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  printf("App_ThreadX_Init: Started\r\n");

  // Allocate stack memory for the thread
     ret = tx_byte_allocate(byte_pool, (VOID **) &i2c_can_thread_stack, 1024, TX_NO_WAIT);
     if (ret != TX_SUCCESS)
     {
     	printf("Stack allocation failed: %u\r\n", ret);
         return TX_POOL_ERROR;
     }
     printf("Stack allocated\r\n");

     /* Init CAN RX module (module will create its own queue and start CAN + notifications) */
       ret = can_rx_init(byte_pool);
       if (ret != TX_SUCCESS)
       {
         printf("can_rx_init failed: %u\r\n", ret);
         return ret;
       }

       /* Create thread */
       ret = tx_thread_create(&i2c_can_thread,
                              "I2C_CAN_Thread",
                              App_ThreadX_Entry,
                              0,
                              i2c_can_thread_stack,
                              1024,
                              1,
                              1,
                              TX_NO_TIME_SLICE,
                              TX_AUTO_START);
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
  * @brief  Function that implements the kernel's initialization.
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
static uint32_t tim2_get_tick_hz(void)
{
    // TIM2 is on APB1
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    // If APB1 prescaler != 1, timer clock = 2 * PCLK1 (STM32 rule)
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t timclk = (ppre1 >= 4U) ? (pclk1 * 2U) : pclk1;

    return timclk / (htim2.Init.Prescaler + 1U);
}

VOID App_ThreadX_Entry(ULONG arg)
{
  (void)arg;

  can_pot_msg_t msg;
  servo_api_t servo;

  uint32_t rx_cnt = 0;
  uint16_t last_val = 0xFFFF;

  uint16_t v = 0;

  // Variables for CAN heart beat check ///
  ULONG last_hb_tick = tx_time_get();
  can_hb_init(&hcan1, 0x476);

//  uint32_t last_isr = 0;		debug for ISR
//  uint32_t last_err = 0;		debug for I2C error

  lcd_init();
  lcd_set_cursor(0,0);
  lcd_print("Pot->Servo");
  lcd_set_cursor(1,0);
  lcd_print("S:--- A:---");

  uint32_t tick_hz = tim2_get_tick_hz();
  servo_api_init(&servo, &htim2, TIM_CHANNEL_1, tick_hz, 500, 2500);

  while (1)
  {
	  // nhận pot (timeout ngắn để còn gửi hb định kỳ)
      if (can_rx_receive(&msg, TX_TIMER_TICKS_PER_SECOND/20) == TX_SUCCESS)		// ~50ms
      {
          rx_cnt++;

          v = (uint16_t)msg.pot_raw;
          uint16_t angle = servo_api_scaled_to_angle(v);

          servo_api_set_angle_deg(&servo, angle);

          /* Only update LCD when value changes */
              if (v != last_val)
              {
                  last_val = v;
                  char line[17];
                  unsigned vv = (unsigned)(v % 1000U);
//                  snprintf(line, sizeof(line), "Scaled:%03u      ", vv);
                  snprintf(line, sizeof(line), "S:%3u A:%3u   ", vv, (unsigned)angle);



				  lcd_set_cursor(1,0);
				  lcd_print(line);
              }

			  if ((rx_cnt % 10U) == 0U)
			  {
				  printf("[APP] rx=%lu pot=%u isr=%lu mis=%lu qfull=%lu\r\n",
						 (unsigned long)rx_cnt,
						 (unsigned)msg.pot_raw,
						 (unsigned long)can_rx_get_isr_hit_cnt(),
						 (unsigned long)can_rx_get_id_mismatch_cnt(),
						 (unsigned long)can_rx_get_q_full_cnt());
			  }
      }

      // gửi heartbeat theo chu kỳ
             ULONG now = tx_time_get();
             if ((now - last_hb_tick) >= (HB_PERIOD_MS * TX_TIMER_TICKS_PER_SECOND) / 1000U)
             {
                 last_hb_tick = now;

                 uint8_t flags = 0;
                 flags |= 0x01; // rx_ok (ví dụ)

                 (void)can_hb_send(v,
                                   flags,
                                   (uint8_t)(can_rx_get_q_full_cnt() & 0xFF),
                                   0 /* nếu bạn có lcd_get_i2c_err() thì đưa vào */);
             }

//      else
//      {
//          uint32_t now_isr = can_rx_get_isr_hit_cnt();
//          if (now_isr != last_isr)
//          {
//              printf("[APP] ISR hit=%lu (but no queue msg?)\r\n", (unsigned long)now_isr);
//              last_isr = now_isr;
//          }
//          /* timeout: không có msg trong 1s */
//          printf("[APP] waiting... isr=%lu\r\n", (unsigned long)now_isr);
//      }

//      /* LCD I2C error counter (debug) */
//      uint32_t err = lcd_get_i2c_err();
//      if (err != last_err)
//      {
//          printf("[LCD] i2c_err=%lu\r\n", (unsigned long)err);
//          last_err = err;
//      }
  }

}
/* USER CODE END 1 */
