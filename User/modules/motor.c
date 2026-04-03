
#include "motor.h"
#include "dwt.h"

static uint8_t idx = 0;
//模块'motor'的私有变量,用于保存电机实例的地址
static Motor_Instance_s *motor_instance[MOTOR_COUNT] = {0};


/**
 * @brief 电机初始化函数
 * 
 */
Motor_Instance_s* Motor_Init(Motor_Init_Config_s *config)
{
    Motor_Instance_s *instance = (Motor_Instance_s *)malloc(sizeof(Motor_Instance_s));
    memset(instance, 0, sizeof(Motor_Instance_s));

    instance->controller = config->controller;
    instance->setting = config->setting;
    HAL_TIM_PWM_Start(instance->setting.pwm_config.htim, instance->setting.pwm_config.channel1);
    HAL_TIM_PWM_Start(instance->setting.pwm_config.htim, instance->setting.pwm_config.channel2);

    motor_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 电机驱动函数,根据输入值控制电机正反转和速度
 * @param value 范围为正负比较寄存器的值,
 */
void MotorDrive(int16_t value, Motor_PWM_Config_s *pwm_config)
{
    if(value > VALUE_COMPARE) value = VALUE_COMPARE;
    if(value < -VALUE_COMPARE) value = -VALUE_COMPARE;

    if(value <= 0)
    {
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel1, -value);
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel1, 0);
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel2, value);
    }
}

void LimitValue(float *value, float max)
{
    if (*value > max) {
        *value = max;
    }
    if (*value < -max) {
        *value = -max;
    }
}


/**
 * @brief 由于用两个PWM控制一个电机,所以这里的速度是-100~100,根据正负号改变PWM信号使能,从而改变方向
 * @brief 目前只有角度单环控制,后面考虑加速度环
 * @param speed 范围-100~100
 */
void MotorTask()
{
    Motor_Instance_s *motor;
    Motor_Setting_s *setting;
    Motor_Controller_s *controller;
    float pid_ref;
    float pid_measure;

    for(int i = 0; i < MOTOR_COUNT; i++)
    {
        if(motor_instance[i] == NULL) break;

        motor = motor_instance[i];
        setting = &motor->setting;
        controller = &motor->controller;
        pid_ref = controller->pid_ref;

        //角度获取
        motor->measures.angle = *motor->setting.ptr_angle;
        // float raw_speed = *motor->setting.ptr_speed;
        if (motor->setting.flag_feedback_reverse == FEEDBACK_DIR_REVERSE) {
                motor->measures.angle *= -1;
            }

        //角度环计算
        if (controller->loop_type & ANGLE_LOOP) {
            pid_measure = motor->measures.angle;
            pid_ref = PIDCalculate(&controller->angle_pid, pid_measure, pid_ref);
        }
        
        if (controller->loop_type & SPEED_LOOP) {
            pid_measure = motor->measures.speed;
            pid_ref = PIDCalculate(&controller->speed_pid, pid_measure, pid_ref);
        }

        //前馈,须在反转判断之前?
        pid_ref += controller->feedforward;

        //限幅
        LimitValue(&pid_ref, controller->angle_pid.maxout);

        if (setting->flag_motor_reverse == MOTOR_DIR_REVERSE) {
            pid_ref *= -1;
        }

        if (motor->setting.motor_state == MOTOR_STOP) {
            pid_ref = 0;
        }

        motor->controller.set = pid_ref;
        
        MotorDrive((int16_t)motor->controller.set, &setting->pwm_config);

    }
}


void MotorSetRef(Motor_Instance_s *motor, float ref)
{
    motor->controller.pid_ref = ref;
}

void MotorEnable(Motor_Instance_s *motor)
{
    motor->setting.motor_state = MOTOR_ENABLE;
}

void MotorStop(Motor_Instance_s *motor)
{
    motor->setting.motor_state = MOTOR_STOP;
}

void MotorSetFeedforward(Motor_Instance_s *motor, float feedforward)
{
    motor->controller.feedforward = feedforward;
}

float MotorGetAngle(const Motor_Instance_s *motor)
{
    return motor->measures.angle;
}


void MotorChangeLoop(Motor_Instance_s *motor, Loop_Type_e loop_type)
{
    motor->controller.loop_type = loop_type;
}
