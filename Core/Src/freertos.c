/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vofa.h"
#include "cmsis_os2.h"
#include "bsp_fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  vofa demo task - sends sinusoidal waveform data via USB CDC
  */
void StartVofaTask(void *argument)
{
  for(;;)
  {
    vofa_demo();
    osDelay(10);
  }
}

/**
  * @brief  CAN1 发送任务 - 每100ms发送递增数据到CAN1
  *         CAN ID: 0x100, 8 bytes
  */
extern FDCAN_HandleTypeDef hfdcan1;

void StartCanDemoTask(void *argument)
{
  uint8_t tx_data[8] = {0};
  uint32_t counter = 0;

  for(;;)
  {
    tx_data[0] = (counter >> 24) & 0xFF;
    tx_data[1] = (counter >> 16) & 0xFF;
    tx_data[2] = (counter >> 8) & 0xFF;
    tx_data[3] = counter & 0xFF;
    tx_data[4] = 0xAA;
    tx_data[5] = 0xBB;
    tx_data[6] = 0xCC;
    tx_data[7] = 0xDD;

    fdcanx_send_data(&hfdcan1, 0x100, tx_data, 8);
    counter++;
    osDelay(100);
  }
}

/**
  * @brief  CAN2 接收任务 - 接收CAN2数据并通过vofa发送
  *         使用TaskNotify等待中断通知，收到后通过vofa输出
  */
void StartCan2RxTask(void *argument)
{
  /* 注册当前任务句柄到FDCAN2接收回调 */
  fdcan2_register_rx_task(xTaskGetCurrentTaskHandle());

  uint16_t rx_id = 0;
  uint8_t rx_buf[8] = {0};
  uint8_t rx_len = 0;

  for(;;)
  {
    /* 阻塞等待FDCAN2接收中断通知 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 读取接收数据 */
    rx_len = fdcan2_get_rx_data(&rx_id, rx_buf);
    if(rx_len > 0)
    {
      /* 通过vofa发送: ch0=ID, ch1-ch4=数据前4字节作为float */
      float id_float = (float)rx_id;
      float d0 = (float)rx_buf[0];
      float d1 = (float)rx_buf[1];
      float d2 = (float)rx_buf[2];
      float d3 = (float)rx_buf[3];

      vofa_send_data(0, id_float);
      vofa_send_data(1, d0);
      vofa_send_data(2, d1);
      vofa_send_data(3, d2);
      vofa_send_data(4, d3);
      vofa_sendframetail();
    }
  }
}

/* USER CODE END Application */

