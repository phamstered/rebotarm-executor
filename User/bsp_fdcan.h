#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

void bsp_can_filter_start(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan2_register_rx_task(TaskHandle_t handle);
uint8_t fdcan2_get_rx_data(uint16_t *id, uint8_t *buf);

#endif /* __BSP_FDCAN_H__ */
