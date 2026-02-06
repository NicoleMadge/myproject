#include "main.h"                       
#include "motor.h"
#include <string.h>

motor_t motor;
void dm_motor_init(void) 
{
    
    memset(&motor, 0, sizeof(motor));
    
    motor.id = 0x01;
    motor.mst_id = 0x11;        // 实际没有用上，只做标识作用
    motor.tmp.read_flag = 1;
    motor.ctrl.mode = MIT_MODE;        
    motor.ctrl.vel_set = 1.0f;  
    motor.ctrl.pos_set = 0.0f;
    motor.ctrl.tor_set = 0.0f;
    motor.ctrl.cur_set = 0.0f;
    motor.ctrl.kp_set = 0.0f;
    motor.ctrl.kd_set = 1.0f;   
    
    motor.tmp.PMAX = 12.5f;
    motor.tmp.VMAX = 30.0f;
    motor.tmp.TMAX = 10.0f;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits) 
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) 
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//使能
uint8_t motor_enable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    
    uint16_t can_id = motor_id + 0x00;// MIT模式偏移ID为0x00(0x11)
    
    return canx_bsp_send_data(hcan, can_id, data, 8);
}

//控制
void motor_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint16_t id = motor_id + 0x00;

	pos_tmp = float_to_uint(pos, -12.5f, 12.5f, 16);   // 位置：±12.5 rad
    vel_tmp = float_to_uint(vel, -30.0f, 30.0f, 12);   // 速度：±30 rad/s (DM4310)
    tor_tmp = float_to_uint(tor, -10.0f, 10.0f, 12);   // 扭矩：±10 Nm (DM4310)
    kp_tmp  = float_to_uint(kp,  0.0f, 500.0f, 12);    // Kp：0~500
    kd_tmp  = float_to_uint(kd,  0.0f, 5.0f, 12);      // Kd：0~5

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp;
    
    canx_bsp_send_data(hcan, id, data, 8);
}

//解析
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0])&0x0F;
    motor->para.state = (rx_data[0])>>4;           
    motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

//失能
uint8_t motor_disable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    
    uint16_t can_id = motor_id + 0x00;
    
    return canx_bsp_send_data(hcan, can_id, data, 8);
}

