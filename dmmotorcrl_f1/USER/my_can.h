#ifndef __MY_CAN_H
#define __MY_CAN_H

#include "main.h"


uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_bsp_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf);
void CAN1_Filter_Config(CAN_HandleTypeDef *hcan);
#endif
