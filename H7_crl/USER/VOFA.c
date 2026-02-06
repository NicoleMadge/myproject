#include "vofa.h"   
#include "usart.h"


float vofa_buff[10];

void Float_to_Byte(float Fdata,  uint8_t *ArrayByte1)
{
    
    Vofa_Type Vofa;                   //定义Vofa_Type类型的Vofa变量
    
    Vofa.Fdata = Fdata;               //把需要操作的浮点数复制到共同体的Fdata变量中
    ArrayByte1[0] = Vofa.Adata;        //0-7位移到数组元素0
    ArrayByte1[1] = Vofa.Adata >> 8;   //8-15位移动到数组元素1
    ArrayByte1[2] = Vofa.Adata >> 16;  //16-23位移动到数组元素2
    ArrayByte1[3] = Vofa.Adata >> 24;  //24-31位移动到数组元素3 
    
}

/**
  * 函数功能：串口发送数据到VOFA+上位机
  * 入口参数：无
  * 返 回 值：无
  */

void JustFloat(float *vofa_buff,uint8_t len)
{
	
    uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7f};    //定义包尾数组
	  uint8_t buffer[100];
		uint8_t lenth=0;
    for(uint8_t i=0;i<len;i++)
	{
		Float_to_Byte(vofa_buff[i],&buffer[lenth]);
		lenth+=4;
	}
	memcpy(&buffer[lenth], Tail, 4);
	lenth+=4;
	HAL_UART_Transmit_DMA(&huart1, buffer, lenth);
}


