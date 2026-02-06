#include "my_usart.h"


volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; 
RC_Ctl_t RC_CtrlData = {0};

 void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{ 
 if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) 
 { 
	  __HAL_DMA_DISABLE(huart->hdmarx); 

	  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT; 
 
	  __HAL_DMA_SET_COUNTER(huart->hdmarx,36); 

	  if(Size == RC_FRAME_LENGTH) 
	  { 
		RemoteDataProcess((uint8_t *)sbus_rx_buffer[0]); 
	  } 
 
 }
 else
{ 
	 __HAL_DMA_DISABLE(huart->hdmarx); 

	  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT); 
 
	  __HAL_DMA_SET_COUNTER(huart->hdmarx,36); 

	  if(Size == RC_FRAME_LENGTH) 
	  { 
		RemoteDataProcess((uint8_t *)sbus_rx_buffer[1]); 
	  }			 
 } 
		 __HAL_DMA_ENABLE(huart->hdmarx);				 
}


void RemoteDataProcess(uint8_t *pData)
{
 if(pData == NULL)
 {
	return;
 }
 
 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; 
 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
 
 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2; 
 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8); 
 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
 RC_CtrlData.mouse.press_l = pData[12]; RC_CtrlData.mouse.press_r = pData[13];
 RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
 
 RC_CtrlData.rc.ch0 -=1024;
 RC_CtrlData.rc.ch1 -=1024;
 RC_CtrlData.rc.ch2 -=1024;
 RC_CtrlData.rc.ch3 -=1024;
}