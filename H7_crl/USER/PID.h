#ifndef __PID_H
#define __PID_H

#include "stdint.h"

// PID控制器结构体
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float target;       // 目标值
    float feedback;     // 反馈值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累积
    float derivative;   // 微分值
    
    float output;       // PID输出
    float max_output;   // 输出限幅最大值
    float min_output;   // 输出限幅最小值
    
    float max_integral; // 积分限幅
    float min_integral; // 积分限幅
    
    uint8_t enable;     // PID使能标志
} PID_TypeDef;

typedef enum 
{
    PID_POSITION = 0,   
    PID_VELOCITY = 1    
 } PID_Mode_TypeDef;

// 函数声明
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
void PID_SetTarget(PID_TypeDef *pid, float target);
void PID_SetFeedback(PID_TypeDef *pid, float feedback);
void PID_SetOutputLimit(PID_TypeDef *pid, float max_output, float min_output);
void PID_SetIntegralLimit(PID_TypeDef *pid, float max_integral, float min_integral);
float PID_Calculate(PID_TypeDef *pid, PID_Mode_TypeDef mode);
void PID_Reset(PID_TypeDef *pid);
void PID_Enable(PID_TypeDef *pid, uint8_t enable);

#endif /* __PID_H */

 