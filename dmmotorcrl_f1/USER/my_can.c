#include "main.h"    
static uint16_t i;
void CAN1_Filter_Config(CAN_HandleTypeDef *hcan)
{
	
    CAN_FilterTypeDef sFilterConfig; // 定义 HAL 库过滤器配置结构体

    // 配置过滤器参数
    sFilterConfig.FilterBank            = 0; // 过滤器编号0
    sFilterConfig.FilterMode            = CAN_FILTERMODE_IDMASK; // 使用标识符掩码模式
    sFilterConfig.FilterScale           = CAN_FILTERSCALE_32BIT; // 使用32位过滤器
    sFilterConfig.FilterIdHigh          = 0x0000; // 设置ID高16位
    sFilterConfig.FilterIdLow           = 0x0000;  // 设置ID低16位
    sFilterConfig.FilterMaskIdHigh      = 0x0000; // 掩码高16位
    sFilterConfig.FilterMaskIdLow       = 0x0000;  // 掩码低16位
    sFilterConfig.FilterFIFOAssignment  = CAN_FILTER_FIFO1; // 过滤器关联到 FIFO0
    sFilterConfig.FilterActivation      = ENABLE; // 激活过滤器
//    sFilterConfig.SlaveStartFilterBank  = 14; // 从过滤器起始编号（用于多CAN）或许不需要

    // 调用 HAL 库函数配置过滤器
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) == HAL_OK)
    {
			i++;
    }
}

uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
//	if(hcan->State != HAL_CAN_STATE_READY)
//	{
//		return 1; 
//	}
	CAN_TxHeaderTypeDef tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = len;
  
	static uint32_t pTxMailbox = 0; 
	
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, &pTxMailbox) != HAL_OK)
	  {
		return 1; 
	  }
}

uint8_t canx_bsp_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf)
{	
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC; 
	}
	else
		return 0;
}

