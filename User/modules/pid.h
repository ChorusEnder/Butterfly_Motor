#ifndef PID_H
#define PID_H

#include "main.h"
#include <math.h>

typedef enum
{
    PID_Improve_none = 0b00000000,
    PID_I_limit = 0b00000001,          // 积分限幅
    PID_D_On_Measurement = 0b00000010, // 0000 0010  微分先行
    PID_T_Intergral = 0b00000100,      // 0000 0100  梯形积分
    PID_P_On_Measurement = 0b00001000, // 0000 1000  比例先行
    PID_OutputFilter = 0b00010000,     // 0001 0000  输出滤波
    PID_Changing_I = 0b00100000,  // 0010 0000  变速积分
    PID_D_Filter = 0b01000000,         // 0100 0000  微分滤波
    PID_ErrorHandle = 0b10000000,      // 1000 0000
} PID_Improve_e;

typedef struct
{
    /*--------------------配置部分-----------------]*/
    float kp;     // 比例系数
    float ki;     // 积分系数
    float kd;     // 微分系数
    float maxout; // 输出限幅
    float deadband;
    // 用于改善pid计算
    PID_Improve_e Improve;
    float i_limit;     // 积分限幅
    float core_a;   // 变速积分,较大误差
    float core_b;   //变速积分,较小误差
    float output_LPF_RC;     // 输出滤波器 RC = 1/omegac dt*10
    float derivative_LPF_RC; // 微分滤波器系数 dt*50
    float feedforword_k;     // 前馈参数

    /*--------------------计算变量-----------------*/
    // 用于计算的变量
    float err[3]; // 误差数组
    float measure;
    float last_measure;
    float last_iterm;
    float last_dout;

    float pout;
    float iout;
    float iterm; // 积分项
    float dout;
    float output; // 输出值

    // 用于增量式pid计算
    float delta_output;

    // 用于积分
    float dt;
    // 用于给时钟计数
    uint32_t DWT_CNT;

} PID_Instance_s;

/**
 * @brief PID计算函数
 * @param pid PID实例
 * @param measure 测量值
 * @param target 目标值
 */
float PIDCalculate(PID_Instance_s *pid, float measure, float target);

#endif