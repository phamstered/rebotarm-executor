#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define BSP_FDCAN_RX_QUEUE_DEPTH 16U

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} BspFdcanFrame;

void bsp_can_filter_start(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

/* FDCAN1 接收接口 */
void fdcan1_register_rx_task(TaskHandle_t handle);
uint8_t fdcan1_get_rx_data(uint16_t *id, uint8_t *buf);
uint32_t fdcan1_get_rx_overflow_count(void);

/* FDCAN2 接收接口 */
void fdcan2_register_rx_task(TaskHandle_t handle);
uint8_t fdcan2_get_rx_data(uint16_t *id, uint8_t *buf);
uint32_t fdcan2_get_rx_overflow_count(void);

#endif /* __BSP_FDCAN_H__ */
