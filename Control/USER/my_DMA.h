#ifndef __MY_DMA_H
#define __MY_DMA_H

#include "main.h"

void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);

#endif 

