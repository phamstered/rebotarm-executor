#include "bsp_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* FDCAN2 接收数据缓存 + 任务通知句柄 */
static uint8_t fdcan2_rx_data[8] = {0};
static uint16_t fdcan2_rx_id = 0;
static uint8_t fdcan2_rx_len = 0;
static volatile uint8_t fdcan2_rx_flag = 0;
static TaskHandle_t fdcan2_rx_task_handle = NULL;

/**
************************************************************************
* @brief:      	bsp_can_filter_start(void)
* @param:       void
* @retval:     	void
* @details:    	配置FDCAN1/FDCAN2滤波器并启动
************************************************************************
**/
void bsp_can_filter_start(void)
{
	FDCAN_FilterTypeDef fdcan_filter;

	/* FDCAN1: 接收所有标准ID */
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x00;
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	/* FDCAN2: 接收所有标准ID */
	fdcan_filter.FilterIndex = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	0成功，1失败
* @details:    	发送Classic CAN数据帧
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = (len <= 8) ? len : 8;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
		return 1;
	return 0;
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       rec_id：接收到的CAN ID
* @param:       buf：接收数据缓存
* @retval:     	接收到的数据长度，0表示无数据
* @details:    	从RX FIFO0接收数据
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;

	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		len = pRxHeader.DataLength;
		return len;
	}
	return 0;
}

/**
************************************************************************
* @brief:      	fdcan2_register_rx_task(TaskHandle_t handle)
* @param:       handle: 需要在FDCAN2接收时通知的FreeRTOS任务句柄
* @retval:     	void
* @details:    	注册任务句柄，FDCAN2收到数据时通过TaskNotify通知该任务
************************************************************************
**/
void fdcan2_register_rx_task(TaskHandle_t handle)
{
	fdcan2_rx_task_handle = handle;
}

/**
************************************************************************
* @brief:      	fdcan2_get_rx_data(uint16_t *id, uint8_t *buf)
* @param:       id: 接收到的CAN ID
* @param:       buf: 接收数据缓存（至少8字节）
* @retval:     	接收到的数据长度，0表示无新数据
* @details:    	获取FDCAN2最新接收数据（从RX回调中读取）
************************************************************************
**/
uint8_t fdcan2_get_rx_data(uint16_t *id, uint8_t *buf)
{
	if(!fdcan2_rx_flag)
		return 0;

	*id = fdcan2_rx_id;
	uint8_t len = fdcan2_rx_len;
	for(uint8_t i = 0; i < len; i++)
		buf[i] = fdcan2_rx_data[i];

	fdcan2_rx_flag = 0;
	return len;
}

/* RX callbacks */
static uint8_t rx_data1[8] = {0};
static uint16_t rec_id1;

static void fdcan1_rx_callback(void)
{
	fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}

static void fdcan2_rx_callback(void)
{
	fdcan2_rx_len = fdcanx_receive(&hfdcan2, &fdcan2_rx_id, fdcan2_rx_data);
	if(fdcan2_rx_len > 0)
	{
		fdcan2_rx_flag = 1;
		/* 通知注册的接收任务 */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if(fdcan2_rx_task_handle != NULL)
		{
			vTaskNotifyGiveFromISR(fdcan2_rx_task_handle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	else if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
}
