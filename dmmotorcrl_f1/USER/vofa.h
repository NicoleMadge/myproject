#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"
#include "my_can.h"   

typedef union
{
    float Fdata;         //以浮点数形式读取变量
    uint32_t Adata;      //以32位无符号形式读取变量
    
}Vofa_Type;

extern float vofa_buff[10];

void Float_to_Byte(float Fdata,  uint8_t *ArrayByte1);
void JustFloat(float *vofa_buff,uint8_t len);
#endif
