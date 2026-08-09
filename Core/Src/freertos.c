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
#include "damiao_protocol.h"
#include "motor_bridge.h"

// micro-ROS includes
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <microros_stm32h723_bridge/msg/joint_command.h>
#include <microros_stm32h723_bridge/msg/hardware_status.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// 电机控制代码已注释，仅保留 micro-ROS 功能
// #include "bsp_fdcan.h"
// #include "damiao_protocol.h"
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
// micro-ROS 传输层函数声明（USB CDC）
extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// micro-ROS 内存分配器声明
extern void * microros_allocate(size_t size, void * state);
extern void microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartMotorControlTask(void *argument)
{
  (void)argument;
  TickType_t wake = xTaskGetTickCount();
  fdcan1_register_rx_task(xTaskGetCurrentTaskHandle());
  motor_bridge_init();
  for (;;) {
    motor_bridge_control_step(HAL_GetTick());
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(2U));
  }
}

void StartSafetyTask(void *argument)
{
  (void)argument;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    motor_bridge_safety_step(HAL_GetTick());
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
  }
}

/**
  * @brief  vofa demo task - 已禁用，USB CDC 供 micro-ROS 使用
  */
// void StartVofaTask(void *argument)
// {
//   for(;;)
//   {
//     vofa_demo();
//     osDelay(10);
//   }
// }

/**
  * @brief  CAN1 发送任务 - 每100ms发送递增数据到CAN1
  *         CAN ID: 0x100, 8 bytes
  */
extern FDCAN_HandleTypeDef hfdcan1;

#define DAMIAO_TEST_MOTOR_ID          0x01U
#define DAMIAO_TEST_KP                8.0f
#define DAMIAO_TEST_KD                2.0f
#define DAMIAO_TEST_ROTATION_RAD      6.283185307f
#define DAMIAO_TEST_RAMP_RAD_S        0.5f
#define DAMIAO_TEST_PERIOD_MS         20U
#define DAMIAO_TEST_FEEDBACK_LIMIT_MS 100U

static const DamiaoLimits damiao_test_limits = {
  .p_min = -12.5f,
  .p_max = 12.5f,
  .v_min = -30.0f,
  .v_max = 30.0f,
  .t_min = -5.0f,
  .t_max = 5.0f
};

static DamiaoMotorState damiao_test_motor;
volatile float damiao_test_phase;
volatile float damiao_test_target;
volatile float damiao_test_vbus;
volatile uint32_t damiao_test_status;

static int damiao_test_can_send(uint16_t can_id, const uint8_t *data, void *ctx)
{
  (void)ctx;
  return fdcanx_send_data(&hfdcan1, can_id, (uint8_t *)data, DAMIAO_DLC) == 0U
      ? 0 : -1;
}

static int damiao_test_can_recv(uint16_t *can_id, uint8_t *data,
                                uint32_t timeout_ms, void *ctx)
{
  (void)ctx;
  uint32_t start = HAL_GetTick();
  for (;;) {
    if (fdcan1_get_rx_data(can_id, data) == DAMIAO_DLC) {
      return 0;
    }
    uint32_t elapsed = HAL_GetTick() - start;
    if (elapsed >= timeout_ms) {
      return -1;
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms - elapsed)) == 0U) {
      return -1;
    }
  }
}

static void damiao_test_sleep(uint32_t ms, void *ctx)
{
  (void)ctx;
  osDelay(ms);
}

static void damiao_test_telemetry(float phase, float target)
{
  damiao_test_phase = phase;
  damiao_test_target = target;
  damiao_test_status = damiao_test_motor.last_feedback.status_code;
  vofa_send_data(0U, phase);
  vofa_send_data(1U, target);
  vofa_send_data(2U, damiao_test_motor.last_feedback.pos);
  vofa_send_data(3U, damiao_test_motor.last_feedback.vel);
  vofa_send_data(4U, damiao_test_motor.last_feedback.torq);
  vofa_sendframetail();
}

static void damiao_test_disable_forever(float error_code)
{
  for (;;) {
    (void)damiao_motor_disable(&damiao_test_motor, damiao_test_can_send, NULL);
    damiao_test_telemetry(error_code, 0.0f);
    osDelay(100U);
  }
}

static bool damiao_test_feedback_safe(float target)
{
  const DamiaoFeedback *feedback = &damiao_test_motor.last_feedback;
  return damiao_test_motor.has_feedback &&
      isfinite(feedback->pos) && isfinite(feedback->vel) &&
      isfinite(feedback->torq) &&
      feedback->status_code == DAMIAO_STATUS_ENABLED &&
      fabsf(feedback->pos) <= 7.0f &&
      fabsf(feedback->pos - target) <= 0.50f &&
      fabsf(feedback->vel) <= 1.50f && fabsf(feedback->torq) <= 1.0f;
}

static bool damiao_test_run_ramp(float phase, float from, float to)
{
  uint8_t command[DAMIAO_DLC];
  uint32_t start = HAL_GetTick();
  float distance = fabsf(to - from);
  uint32_t duration_ms = (uint32_t)(distance / DAMIAO_TEST_RAMP_RAD_S * 1000.0f);

  while ((uint32_t)(HAL_GetTick() - start) <= duration_ms) {
    uint32_t cycle_start = HAL_GetTick();
    float progress = duration_ms == 0U ? 1.0f :
        (float)(cycle_start - start) / (float)duration_ms;
    if (progress > 1.0f) progress = 1.0f;
    float target = from + (to - from) * progress;
    damiao_encode_mit_cmd(target, 0.0f, 0.0f, DAMIAO_TEST_KP, DAMIAO_TEST_KD,
                          &damiao_test_limits, command);
    if (damiao_test_can_send(DAMIAO_TEST_MOTOR_ID, command, NULL) != 0) {
      return false;
    }

    bool received = false;
    uint32_t feedback_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - feedback_start) < DAMIAO_TEST_FEEDBACK_LIMIT_MS) {
      uint16_t can_id;
      uint8_t data[DAMIAO_DLC];
      if (damiao_test_can_recv(&can_id, data, 10U, NULL) == 0) {
        damiao_motor_state_process_frame(&damiao_test_motor, can_id, data);
        if (can_id == damiao_test_motor.feedback_id) {
          received = true;
          break;
        }
      }
    }
    if (!received || !damiao_test_feedback_safe(target)) {
      return false;
    }
    damiao_test_telemetry(phase, target);

    uint32_t elapsed = HAL_GetTick() - cycle_start;
    if (elapsed < DAMIAO_TEST_PERIOD_MS) {
      osDelay(DAMIAO_TEST_PERIOD_MS - elapsed);
    }
  }
  return true;
}

static bool damiao_test_hold(float phase, float target, uint32_t duration_ms)
{
  uint8_t command[DAMIAO_DLC];
  uint32_t start = HAL_GetTick();
  damiao_encode_mit_cmd(target, 0.0f, 0.0f, DAMIAO_TEST_KP, DAMIAO_TEST_KD,
                        &damiao_test_limits, command);
  while ((uint32_t)(HAL_GetTick() - start) < duration_ms) {
    if (damiao_test_can_send(DAMIAO_TEST_MOTOR_ID, command, NULL) != 0) {
      return false;
    }
    bool received = false;
    uint32_t feedback_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - feedback_start) <
           DAMIAO_TEST_FEEDBACK_LIMIT_MS) {
      uint16_t can_id;
      uint8_t data[DAMIAO_DLC];
      if (damiao_test_can_recv(&can_id, data, 10U, NULL) == 0) {
        damiao_motor_state_process_frame(&damiao_test_motor, can_id, data);
        if (can_id == damiao_test_motor.feedback_id) {
          received = true;
          break;
        }
      }
    }
    if (!received || !damiao_test_feedback_safe(target)) {
      return false;
    }
    damiao_test_telemetry(phase, target);
    osDelay(DAMIAO_TEST_PERIOD_MS);
  }
  return true;
}

static bool damiao_test_wait_status(uint8_t expected_status,
                                    uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start) < timeout_ms) {
    uint16_t can_id;
    uint8_t data[DAMIAO_DLC];
    if (damiao_test_can_recv(&can_id, data, 10U, NULL) == 0 &&
        can_id == damiao_test_motor.feedback_id) {
      damiao_motor_state_process_frame(&damiao_test_motor, can_id, data);
      if (damiao_test_motor.last_feedback.status_code == expected_status) {
        return true;
      }
    }
  }
  return false;
}

void StartDamiaoMotorTask(void *argument)
{
  (void)argument;
  uint8_t command[DAMIAO_DLC];
  uint16_t can_id;
  uint8_t data[DAMIAO_DLC];

  fdcan1_register_rx_task(xTaskGetCurrentTaskHandle());
  damiao_motor_state_init(&damiao_test_motor, DAMIAO_TEST_MOTOR_ID,
                          DAMIAO_TEST_MOTOR_ID + 0x10U, &damiao_test_limits);
  DamiaoModeSwitchCtx mode_ctx = {
    .motor_id = DAMIAO_TEST_MOTOR_ID,
    .send = damiao_test_can_send,
    .recv = damiao_test_can_recv,
    .sleep = damiao_test_sleep,
    .user_ctx = NULL,
    .limits = &damiao_test_limits
  };

  osDelay(1000U);
  (void)damiao_motor_disable(&damiao_test_motor, damiao_test_can_send, NULL);
  osDelay(50U);
  while (fdcan1_get_rx_data(&can_id, data) == DAMIAO_DLC) { }

  if (damiao_motor_request_feedback_state(&damiao_test_motor, &mode_ctx, 300U) !=
          DAMIAO_ERR_OK ||
      damiao_test_motor.last_feedback.status_code != DAMIAO_STATUS_DISABLED ||
      !isfinite(damiao_test_motor.last_feedback.pos) ||
      !isfinite(damiao_test_motor.last_feedback.vel) ||
      !isfinite(damiao_test_motor.last_feedback.torq) ||
      fabsf(damiao_test_motor.last_feedback.pos) > 12.5f ||
      fabsf(damiao_test_motor.last_feedback.vel) > 0.20f ||
      fabsf(damiao_test_motor.last_feedback.torq) > 1.0f) {
    damiao_test_disable_forever(-11.0f);
  }
  if (damiao_ensure_control_mode(&mode_ctx, DAMIAO_MODE_MIT, 1000U) !=
      DAMIAO_ERR_OK) {
    damiao_test_disable_forever(-12.0f);
  }

  uint32_t vbus_raw = 0U;
  if (damiao_read_register_u32(&mode_ctx, 60U, &vbus_raw, 300U) ==
      DAMIAO_ERR_OK) {
    memcpy((void *)&damiao_test_vbus, &vbus_raw, sizeof(vbus_raw));
  } else {
    damiao_test_vbus = -1.0f;
  }

  if (damiao_motor_disable(&damiao_test_motor, damiao_test_can_send, NULL) !=
      DAMIAO_ERR_OK) {
    damiao_test_disable_forever(-13.0f);
  }
  bool zeroed = false;
  for (uint32_t attempt = 0U; attempt < 5U && !zeroed; attempt++) {
    if (damiao_motor_set_zero_position(&damiao_test_motor,
                                       damiao_test_can_send,
                                       damiao_test_sleep, NULL) !=
        DAMIAO_ERR_OK) {
      break;
    }
    osDelay(50U);
    if (damiao_motor_request_feedback_state(&damiao_test_motor, &mode_ctx,
                                            300U) == DAMIAO_ERR_OK &&
        damiao_test_motor.last_feedback.status_code == DAMIAO_STATUS_DISABLED &&
        fabsf(damiao_test_motor.last_feedback.pos) <= 0.05f) {
      zeroed = true;
    }
  }
  if (!zeroed) {
    damiao_test_disable_forever(-13.0f);
  }
  damiao_test_telemetry(1.0f, 0.0f);

  damiao_encode_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        &damiao_test_limits, command);
  if (damiao_test_can_send(DAMIAO_TEST_MOTOR_ID, command, NULL) != 0) {
    damiao_test_disable_forever(-14.0f);
  }
  bool enabled = false;
  for (uint32_t attempt = 0U; attempt < 5U && !enabled; attempt++) {
    if (damiao_motor_enable(&damiao_test_motor, damiao_test_can_send, NULL) !=
        DAMIAO_ERR_OK) {
      break;
    }
    osDelay(50U);
    if (damiao_motor_request_feedback_state(&damiao_test_motor, &mode_ctx,
                                            300U) == DAMIAO_ERR_OK &&
        damiao_test_motor.last_feedback.status_code == DAMIAO_STATUS_ENABLED) {
      enabled = true;
    }
  }
  if (!enabled) {
    damiao_test_disable_forever(-15.0f);
  }
  damiao_test_telemetry(2.0f, 0.0f);

  if (!damiao_test_hold(2.0f, 0.0f, 500U)) {
    damiao_test_disable_forever(-16.0f);
  }
  if (!damiao_test_run_ramp(3.0f, 0.0f, DAMIAO_TEST_ROTATION_RAD)) {
    damiao_test_disable_forever(-17.0f);
  }
  if (!damiao_test_hold(4.0f, DAMIAO_TEST_ROTATION_RAD, 500U)) {
    damiao_test_disable_forever(-18.0f);
  }
  if (!damiao_test_run_ramp(5.0f, DAMIAO_TEST_ROTATION_RAD, 0.0f)) {
    damiao_test_disable_forever(-19.0f);
  }
  if (!damiao_test_hold(6.0f, 0.0f, 2000U) ||
      fabsf(damiao_test_motor.last_feedback.pos) > 0.05f) {
    damiao_test_disable_forever(-20.0f);
  }

  bool disabled = false;
  for (uint32_t attempt = 0U; attempt < 5U && !disabled; attempt++) {
    while (fdcan1_get_rx_data(&can_id, data) == DAMIAO_DLC) { }
    if (damiao_motor_disable(&damiao_test_motor, damiao_test_can_send, NULL) !=
        DAMIAO_ERR_OK) {
      break;
    }
    disabled = damiao_test_wait_status(DAMIAO_STATUS_DISABLED, 200U);
  }
  if (!disabled) {
    damiao_test_disable_forever(-21.0f);
  }
  damiao_test_disable_forever(7.0f);
}
//
// void StartCanDemoTask(void *argument)
// {
//   uint8_t tx_data[8] = {0};
//   uint32_t counter = 0;
//
//   for(;;)
//   {
//     tx_data[0] = (counter >> 24) & 0xFF;
//     tx_data[1] = (counter >> 16) & 0xFF;
//     tx_data[2] = (counter >> 8) & 0xFF;
//     tx_data[3] = counter & 0xFF;
//     tx_data[4] = 0xAA;
//     tx_data[5] = 0xBB;
//     tx_data[6] = 0xCC;
//     tx_data[7] = 0xDD;
//
//     fdcanx_send_data(&hfdcan1, 0x100, tx_data, 8);
//     counter++;
//     osDelay(100);
//   }
// }

/**
  * @brief  CAN2 接收任务 - 接收CAN2数据并通过vofa发送
  *         使用TaskNotify等待中断通知，收到后通过vofa输出
  */
// void StartCan2RxTask(void *argument)
// {
//   /* 注册当前任务句柄到FDCAN2接收回调 */
//   fdcan2_register_rx_task(xTaskGetCurrentTaskHandle());
//
//   uint16_t rx_id = 0;
//   uint8_t rx_buf[8] = {0};
//   uint8_t rx_len = 0;
//
//   for(;;)
//   {
//     /* 阻塞等待FDCAN2接收中断通知 */
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//     /* 读取接收数据 */
//     rx_len = fdcan2_get_rx_data(&rx_id, rx_buf);
//     if(rx_len > 0)
//     {
//       /* 通过vofa发送: ch0=ID, ch1-ch4=数据前4字节作为float */
//       float id_float = (float)rx_id;
//       float d0 = (float)rx_buf[0];
//       float d1 = (float)rx_buf[1];
//       float d2 = (float)rx_buf[2];
//       float d3 = (float)rx_buf[3];
//
//       vofa_send_data(0, id_float);
//       vofa_send_data(1, d0);
//       vofa_send_data(2, d1);
//       vofa_send_data(3, d2);
//       vofa_send_data(4, d3);
//       vofa_sendframetail();
//     }
//   }
// }

/**
  * @brief  Damiao 电机控制任务 - 已注释，仅保留 micro-ROS 功能
  */
// #define DAMIAO_MOTOR_ID     0x01
// #define DAMIAO_MOVE_DEG     360.0f
// #define DAMIAO_MOVE_RAD     (DAMIAO_MOVE_DEG * 3.14159265f / 180.0f)
//
// /* MIT 控制参数 */
// #define MIT_KP              10.0f
// #define MIT_KD              2.0f
// #define MIT_TAU_FF          0 /* 前馈力矩，补偿负载 */
//
// /* CAN 收发回调函数 */
// static int damiao_can_send(uint16_t can_id, const uint8_t *data, void *ctx)
// {
//   (void)ctx;
//   return fdcanx_send_data(&hfdcan1, can_id, (uint8_t *)data, DAMIAO_DLC) == 0 ? 0 : -1;
// }
//
// static int damiao_can_recv(uint16_t *can_id_out, uint8_t *data,
//                            uint32_t timeout_ms, void *ctx)
// {
//   (void)ctx;
//   /* 使用 TaskNotify 等待接收 */
//   if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0) {
//     uint8_t len = fdcan1_get_rx_data(can_id_out, data);
//     return len > 0 ? 0 : -1;
//   }
//   return -1;
// }
//
// static void damiao_sleep(uint32_t ms, void *ctx)
// {
//   (void)ctx;
//   osDelay(ms);
// }
//
// /* 电机限制参数 */
// static const DamiaoLimits damiao_limits = {
//   .p_min = -12.5f,
//   .p_max = 12.5f,
//   .v_min = -30.0f,
//   .v_max = 30.0f,
//   .t_min = -5.0f,
//   .t_max = 5.0f
// };
//
// /* 电机状态 */
// static DamiaoMotorState motor_state;
//
// /* 模式切换上下文 */
// static DamiaoModeSwitchCtx mode_ctx;
//
// void StartDamiaoMotorTask(void *argument)
// {
//   uint8_t tx_data[8] = {0};
//   int rc;
//   uint32_t loop_count = 0;
//
//   /* 注册 FDCAN1 接收任务 */
//   fdcan1_register_rx_task(xTaskGetCurrentTaskHandle());
//
//   /* 初始化电机状态 - feedback_id = motor_id + 0x10 */
//   damiao_motor_state_init(&motor_state, DAMIAO_MOTOR_ID, DAMIAO_MOTOR_ID + 0x10, &damiao_limits);
//
//   /* 初始化模式切换上下文 */
//   mode_ctx.motor_id = DAMIAO_MOTOR_ID;
//   mode_ctx.send = damiao_can_send;
//   mode_ctx.recv = damiao_can_recv;
//   mode_ctx.sleep = damiao_sleep;
//   mode_ctx.user_ctx = NULL;
//   mode_ctx.limits = &damiao_limits;
//
//   /* 等待系统稳定 */
//   osDelay(1000);
//
//   /* 1. 启动电机 */
//   rc = damiao_motor_enable(&motor_state, damiao_can_send, NULL);
//   if (rc != DAMIAO_ERR_OK) {
//     /* 调试: enable 失败 */
//     // vofa_send_data(0, -1.0f);
//     // vofa_send_data(1, (float)rc);
//     // vofa_sendframetail();
//     osDelay(1000);
//   }
//   osDelay(500);  /* 等待电机启动 */
//
//   /* 2. 设置零点 */
//   rc = damiao_motor_set_zero_position(&motor_state, damiao_can_send, damiao_sleep, NULL);
//   if (rc != DAMIAO_ERR_OK) {
//     /* 调试: set_zero 失败 */
//     // vofa_send_data(0, -2.0f);
//     // vofa_send_data(1, (float)rc);
//     // vofa_sendframetail();
//     osDelay(1000);
//   }
//   osDelay(100);
//
//   /* 3. 切换到 MIT 模式 */
//   {
//     uint8_t cmd[8];
//
//     /* 尝试写 CTRL_MODE 寄存器 (RID=10) 切换到 MIT 模式 */
//     damiao_encode_register_write_cmd(DAMIAO_MOTOR_ID, 10, 1, cmd);
//     fdcanx_send_data(&hfdcan1, damiao_get_register_can_id(), cmd, DAMIAO_DLC);
//     osDelay(100);
//
//     /* 发送 clear channels 命令 - 清除所有通道 */
//     /* VEL 通道 */
//     damiao_encode_vel_cmd(0.0f, cmd);
//     fdcanx_send_data(&hfdcan1, damiao_get_vel_can_id(DAMIAO_MOTOR_ID), cmd, DAMIAO_DLC);
//     osDelay(20);
//
//     /* POS_VEL 通道 */
//     damiao_encode_pos_vel_cmd(0.0f, 0.0f, cmd);
//     fdcanx_send_data(&hfdcan1, damiao_get_pos_vel_can_id(DAMIAO_MOTOR_ID), cmd, DAMIAO_DLC);
//     osDelay(20);
//
//     /* FORCE_POS 通道 */
//     damiao_encode_force_pos_cmd(0.0f, 0.0f, 0.0f, cmd);
//     fdcanx_send_data(&hfdcan1, damiao_get_force_pos_can_id(DAMIAO_MOTOR_ID), cmd, DAMIAO_DLC);
//     osDelay(20);
//
//     /* MIT 通道 - 初始位置 0 */
//     damiao_encode_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &damiao_limits, tx_data);
//     fdcanx_send_data(&hfdcan1, damiao_get_mit_can_id(DAMIAO_MOTOR_ID), tx_data, DAMIAO_DLC);
//     osDelay(100);
//   }
//
//   for(uint32_t i = 0; i < 10; i++)
//   {
//     loop_count++;
//
//     /* 5. MIT控制移动30度 */
//     damiao_encode_mit_cmd(DAMIAO_MOVE_RAD, 0.0f, MIT_TAU_FF, MIT_KP, MIT_KD, &damiao_limits, tx_data);
//     fdcanx_send_data(&hfdcan1, damiao_get_mit_can_id(DAMIAO_MOTOR_ID), tx_data, DAMIAO_DLC);
//
//     /* 等待电机移动并收集反馈 */
//     for (int i = 0; i < 20; i++) {
//       /* 持续发送 MIT 命令保持控制 */
//       fdcanx_send_data(&hfdcan1, damiao_get_mit_can_id(DAMIAO_MOTOR_ID), tx_data, DAMIAO_DLC);
//
//       /* 等待电机主动反馈 - 反馈CAN ID = motor_id + 0x10 */
//       if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) > 0) {
//         uint16_t recv_id;
//         uint8_t recv_data[8];
//         uint8_t len = fdcan1_get_rx_data(&recv_id, recv_data);
//         if (len > 0 && recv_id == (DAMIAO_MOTOR_ID + 0x10)) {
//           /* 解析反馈数据 */
//           damiao_decode_sensor_feedback(recv_data, &damiao_limits, &motor_state.last_feedback);
//           motor_state.has_feedback = true;
//         }
//       }
//       osDelay(50);
//     }
//
//     /* 调试: 显示当前状态（vofa 已禁用） */
//     // vofa_send_data(0, (float)loop_count);    /* 循环计数 */
//     // vofa_send_data(1, DAMIAO_MOVE_RAD);     /* 目标位置 */
//     // vofa_send_data(2, motor_state.last_feedback.pos);  /* 实际位置 */
//     // vofa_send_data(3, motor_state.last_feedback.vel);  /* 实际速度 */
//     // vofa_send_data(4, motor_state.last_feedback.torq); /* 实际力矩 */
//     // vofa_sendframetail();
//
//     osDelay(1000);  /* 保持剩余时间 */
//
//     /* 6. 归零 - MIT控制回到0度 */
//     damiao_encode_mit_cmd(0.0f, 0.0f, 0.0f, MIT_KP, MIT_KD, &damiao_limits, tx_data);
//     for (int i = 0; i < 40; i++) {
//       fdcanx_send_data(&hfdcan1, damiao_get_mit_can_id(DAMIAO_MOTOR_ID), tx_data, DAMIAO_DLC);
//       osDelay(50);
//     }
//   }
//
//   /* 10次循环后停止电机 */
//   damiao_motor_disable(&motor_state, damiao_can_send, NULL);
// }

static rcl_publisher_t joint_state_publisher;
static rcl_publisher_t status_publisher;
static sensor_msgs__msg__JointState joint_state_message;
static microros_stm32h723_bridge__msg__HardwareStatus status_message;
static microros_stm32h723_bridge__msg__JointCommand command_message;

static void joint_command_callback(const void *message)
{
  const microros_stm32h723_bridge__msg__JointCommand *ros_command = message;
  MotorBridgeCommand command = {0};
  command.sequence = ros_command->sequence;
  command.mode = ros_command->mode;
  for (uint32_t i = 0U; i < MOTOR_BRIDGE_JOINT_COUNT; i++) {
    command.position[i] = ros_command->position[i];
    command.velocity[i] = ros_command->velocity[i];
    command.kp[i] = ros_command->kp[i];
    command.kd[i] = ros_command->kd[i];
    command.effort[i] = ros_command->effort[i];
  }
  (void)motor_bridge_accept_command(&command, HAL_GetTick());
}

static void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  MotorBridgeSnapshot snapshot;
  motor_bridge_get_snapshot(&snapshot);
  joint_state_message.position.data[0] = snapshot.position;
  joint_state_message.velocity.data[0] = snapshot.velocity;
  joint_state_message.effort.data[0] = snapshot.effort;
  status_message.last_command_sequence = snapshot.last_command_sequence;
  status_message.mode = snapshot.mode;
  status_message.active_joint_mask = snapshot.active_joint_mask;
  status_message.fault_code = snapshot.fault_code;

  int64_t epoch_ms = rmw_uros_epoch_millis();
  if (epoch_ms > 0) {
    int32_t sec = (int32_t)(epoch_ms / 1000LL);
    uint32_t nanosec = (uint32_t)((epoch_ms % 1000LL) * 1000000LL);
    joint_state_message.header.stamp.sec = sec;
    joint_state_message.header.stamp.nanosec = nanosec;
    status_message.header.stamp.sec = sec;
    status_message.header.stamp.nanosec = nanosec;
  }

  if (rcl_publish(&joint_state_publisher, &joint_state_message, NULL) != RCL_RET_OK ||
      rcl_publish(&status_publisher, &status_message, NULL) != RCL_RET_OK) {
    motor_bridge_force_disable(MOTOR_FAULT_NONE);
  }
}

static bool init_ros_messages(void)
{
  if (!sensor_msgs__msg__JointState__init(&joint_state_message) ||
      !microros_stm32h723_bridge__msg__HardwareStatus__init(&status_message) ||
      !microros_stm32h723_bridge__msg__JointCommand__init(&command_message)) {
    return false;
  }
  if (!rosidl_runtime_c__String__Sequence__init(&joint_state_message.name, 1U) ||
      !rosidl_runtime_c__float64__Sequence__init(&joint_state_message.position, 1U) ||
      !rosidl_runtime_c__float64__Sequence__init(&joint_state_message.velocity, 1U) ||
      !rosidl_runtime_c__float64__Sequence__init(&joint_state_message.effort, 1U)) {
    return false;
  }
  return rosidl_runtime_c__String__assign(&joint_state_message.name.data[0], "joint1") &&
         rosidl_runtime_c__String__assign(&joint_state_message.header.frame_id, "base_link") &&
         rosidl_runtime_c__String__assign(&status_message.header.frame_id, "base_link");
}

void StartMicrorosTask(void *argument)
{
  (void)argument;
  // micro-ROS 配置 - 使用 USB CDC 传输层
  rmw_uros_set_custom_transport(
    true,
    NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  // 设置 FreeRTOS 内存分配器
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    for(;;) { osDelay(1000); }
  }

  if (!init_ros_messages()) {
    motor_bridge_force_disable(MOTOR_FAULT_NONE);
    for (;;) { osDelay(1000U); }
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  uint32_t reconnect_delay_ms = 250U;
  for (;;) {
    if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
      motor_bridge_force_disable(MOTOR_FAULT_NONE);
      osDelay(reconnect_delay_ms);
      if (reconnect_delay_ms < 2000U) reconnect_delay_ms *= 2U;
      continue;
    }

    rclc_support_t support = {0};
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    joint_state_publisher = rcl_get_zero_initialized_publisher();
    status_publisher = rcl_get_zero_initialized_publisher();
    bool initialized = false;
    bool support_ready = false;
    bool node_ready = false;
    bool subscription_ready = false;
    bool joint_publisher_ready = false;
    bool status_publisher_ready = false;
    bool timer_ready = false;
    bool executor_ready = false;

    support_ready = rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK;
    if (support_ready) {
      node_ready = rclc_node_init_default(
        &node, "stm32h723_motor_bridge", "", &support) == RCL_RET_OK;
    }
    if (node_ready) {
      subscription_ready = rclc_subscription_init_best_effort(
          &subscription, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(microros_stm32h723_bridge, msg, JointCommand),
          "/stm32/joint_commands") == RCL_RET_OK;
      joint_publisher_ready = rclc_publisher_init_best_effort(
          &joint_state_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "/stm32/joint_states") == RCL_RET_OK;
      status_publisher_ready = rclc_publisher_init_best_effort(
          &status_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(microros_stm32h723_bridge, msg, HardwareStatus),
          "/stm32/status") == RCL_RET_OK;
      timer_ready = rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(20U), publish_timer_callback) == RCL_RET_OK;
    }
    if (subscription_ready && joint_publisher_ready && status_publisher_ready && timer_ready) {
      executor_ready = rclc_executor_init(
        &executor, &support.context, 2U, &allocator) == RCL_RET_OK;
    }
    if (executor_ready &&
        rclc_executor_add_subscription(&executor, &subscription, &command_message,
                                       joint_command_callback, ON_NEW_DATA) == RCL_RET_OK &&
        rclc_executor_add_timer(&executor, &timer) == RCL_RET_OK) {
      initialized = true;
      reconnect_delay_ms = 250U;
      (void)rmw_uros_sync_session(1000);
    }

    uint32_t last_ping_ms = HAL_GetTick();
    while (initialized) {
      if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10U)) != RCL_RET_OK) break;
      if ((uint32_t)(HAL_GetTick() - last_ping_ms) >= 1000U) {
        last_ping_ms = HAL_GetTick();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) break;
      }
      osDelay(1U);
    }

    motor_bridge_force_disable(MOTOR_FAULT_NONE);
    rcl_ret_t fini_result = RCL_RET_OK;
    if (support_ready) {
      rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
      if (rmw_context != NULL) {
        rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0U);
      }
    }
    if (executor_ready) fini_result = rclc_executor_fini(&executor);
    if (timer_ready) fini_result = rcl_timer_fini(&timer);
    if (subscription_ready) fini_result = rcl_subscription_fini(&subscription, &node);
    if (status_publisher_ready) fini_result = rcl_publisher_fini(&status_publisher, &node);
    if (joint_publisher_ready) fini_result = rcl_publisher_fini(&joint_state_publisher, &node);
    if (node_ready) fini_result = rcl_node_fini(&node);
    if (support_ready) fini_result = rclc_support_fini(&support);
    (void)fini_result;
    osDelay(reconnect_delay_ms);
    if (reconnect_delay_ms < 2000U) reconnect_delay_ms *= 2U;
  }
}

/* USER CODE END Application */

