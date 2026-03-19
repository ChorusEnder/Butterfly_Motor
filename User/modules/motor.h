#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"
#include <stdlib.h>
#include <string.h>

//比较寄存器的值,根据实际设置更改
#define VALUE_COMPARE 1000 -1
#define MOTOR_COUNT 4

#ifndef ANG_TO_RAD
#define ANG_TO_RAD 3.1416 / 180.0f //角度转弧度
#endif

#ifndef RAD_TO_ANG
#define RAD_TO_ANG 180.0f / 3.1416f //弧度转角度
#endif

typedef enum {
    MOTOR_DIR_NORMAL = 0,
    MOTOR_DIR_REVERSE = 1,
} Motor_Reverse_Flag_e;

typedef enum {
    FEEDBACK_DIR_NORMAL = 0,
    FEEDBACK_DIR_REVERSE = 1,
} Motor_Feedback_Reverse_Flag_e;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ENABLE,
} Motor_State_e;

typedef enum {
    OPEN_LOOP = 0b0000,
    SPEED_LOOP = 0b0001,
    ANGLE_LOOP = 0b0010,

    SPEED_AND_ANGLE_LOOP = 0b0011,
} Loop_Type_e;

//pwm时钟,通道配置
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel1;
    uint32_t channel2;
} Motor_PWM_Config_s;

//电机测量值
typedef struct {
    float angle;
    float speed;
    float angle_last;
    float dt;
    uint32_t last_cnt; 
} Motor_Measures_s;

//电机控制器
typedef struct {
    float pid_ref;
    float set;
    Loop_Type_e loop_type;
    PID_Instance_s angle_pid;
    PID_Instance_s speed_pid;
    float feedforward;

} Motor_Controller_s;

//电机设置
typedef struct {

    Motor_PWM_Config_s pwm_config;
    Motor_Reverse_Flag_e flag_motor_reverse; // 反转标志
    Motor_Feedback_Reverse_Flag_e flag_feedback_reverse; // 反馈反转标志
    Motor_State_e motor_state;
    float motor_offset;

    float *ptr_angle;//指向反馈角度的指针
    float *ptr_speed;//指向反馈速度的指针
    
} Motor_Setting_s;


typedef struct {
    Motor_Setting_s setting;
    Motor_Measures_s measures;
    Motor_Controller_s controller;
} Motor_Instance_s;

typedef struct {
    Motor_Controller_s controller;
    Motor_Setting_s setting;
} Motor_Init_Config_s;


Motor_Instance_s* Motor_Init(Motor_Init_Config_s *config);
void MotorTask();

//设定电机输出ref
void MotorSetRef(Motor_Instance_s *motor, float ref);

//使能电机
void MotorEnable(Motor_Instance_s *motor);

//停止电机
void MotorStop(Motor_Instance_s *motor);

//设置电机反馈角度方向
void MotorSetFeedforward(Motor_Instance_s *motor, float feedforward);

//获取当前电机角度
float MotorGetAngle(const Motor_Instance_s *motor);

//改变控制环
void MotorChangeLoop(Motor_Instance_s *motor, Loop_Type_e loop_type);

#endif