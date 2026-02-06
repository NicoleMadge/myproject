#include "my_DMA.h"

void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
	 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;//空闲中断接收模式

	 huart->RxEventType = HAL_UART_RXEVENT_IDLE;//中断触发事件类型为「空闲帧中断（IDLE）」

	 huart->RxXferSize = DataLength;

	 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);//硬件层面使能

	 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
	 
	do{
				__HAL_DMA_DISABLE(huart->hdmarx);
		}while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);//确保配置前 DMA 处于非工作状态

	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR=(uint32_t)SecondMemAddress;

	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

	SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(huart->hdmarx);	
}

