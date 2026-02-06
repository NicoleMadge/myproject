#include "pid.h"
#include <math.h>

/**
 * @brief  PID控制器初始化
 * @param  pid: PID结构体指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval None
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    
    // 默认输出限幅
    pid->max_output = 5.0f;
    pid->min_output = -5.0f;
    
    // 默认积分限幅
    pid->max_integral = 1.0f;
    pid->min_integral = -1.0f;
    
    pid->enable = 1;
}

/**
 * @brief  设置PID目标值
 * @param  pid: PID结构体指针
 * @param  target: 目标值
 * @retval None
 */
void PID_SetTarget(PID_TypeDef *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  设置PID反馈值
 * @param  pid: PID结构体指针
 * @param  feedback: 反馈值
 * @retval None
 */
void PID_SetFeedback(PID_TypeDef *pid, float feedback)
{
    pid->feedback = feedback;
}

/**
 * @brief  设置PID输出限幅
 * @param  pid: PID结构体指针
 * @param  max_output: 最大输出值
 * @param  min_output: 最小输出值
 * @retval None
 */
void PID_SetOutputLimit(PID_TypeDef *pid, float max_output, float min_output)
{
    pid->max_output = max_output;
    pid->min_output = min_output;
}

/**
 * @brief  设置PID积分限幅
 * @param  pid: PID结构体指针
 * @param  max_integral: 最大积分值
 * @param  min_integral: 最小积分值
 * @retval None
 */
void PID_SetIntegralLimit(PID_TypeDef *pid, float max_integral, float min_integral)
{
    pid->max_integral = max_integral;
    pid->min_integral = min_integral;
}
/**
 * @brief  PID计算函数
 * @param  pid: PID结构体指针
 * @param  mode: PID模式 (位置式/速度式)
 * @retval PID输出值
 */
float PID_Calculate(PID_TypeDef *pid, PID_Mode_TypeDef mode)
{
    
    // 计算误差
    pid->error = pid->target - pid->feedback;
    
    if (mode == PID_POSITION)
		{
        // 位置式PID
        // 积分项累积
        pid->integral += pid->error;
        
        // 积分限幅
        if (pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if (pid->integral < pid->min_integral) {
            pid->integral = pid->min_integral;
        }
        
        // 微分项
        pid->derivative = pid->error - pid->last_error;
        
        // PID输出计算
        pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
		} 
	else 
	{
    // 速度式PID (增量式)
    pid->derivative = pid->error - pid->last_error;
    
    // 增量式PID输出
    float delta_output = pid->kp * (pid->error - pid->last_error) + 
                        pid->ki * pid->error + 
                        pid->kd * (pid->error - 2 * pid->last_error + pid->integral);
    
    pid->output += delta_output;
    
    // 更新积分项为上上次误差
    pid->integral = pid->last_error;
	}
    
    // 输出限幅
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }
    
    // 更新上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief  PID复位函数
 * @param  pid: PID结构体指针
 * @retval None
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

