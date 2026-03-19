#include "tim.h"
#include "arm_math.h"
#include "bsp_adc.h"
#include "butterfly_task.h"
#include "butterfly.h"
#include "motor.h"
#include "remote_fs.h"
#include "elrs.h"
#include "daemon.h"
#include "bsp_timer.h"
#include "dwt.h"

static butterfly_mode_e butterfly_mode;
static Motor_Instance_s* motor_l;
static Motor_Instance_s* motor_r;
static ELRS_Data *rc_elrs;
static uint16_t *ptr_adc;
static float angle_feedforward_l;
static float angle_feedforward_r;

/*-------------------以下是Asin(wt)+B有关的参数---------*/
static float angle_l;
static float angle_r;
static float time;
static float w = 8;//角速度,单位:rad/s
static float Al = 100;
static float bl = 20;
static float Ar = 100;
static float br = 20;
/*----------------------------------------------------*/

void Butterfly_Init()
{
    OSTask_Init();
    DWT_Init(64);
    ptr_adc = BSP_ADC_Init(&hadc1);
    rc_elrs = REMOTE_ELRS_Init(&huart1);

    Motor_Init_Config_s motorConfig = {
        .controller = {
            // .loop_type = ANGLE_LOOP | SPEED_LOOP,
            .loop_type = OPEN_LOOP,
            .pid_ref = 0.0f,
            .angle_pid = {
                .kp = 5.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .deadband = 1.0f,
                .maxout = 700,
                .Improve = PID_T_Intergral | PID_I_limit | PID_D_Filter | PID_OutputFilter | PID_Changing_I,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 20.0f,
            },
            .speed_pid = {
                .kp = 0.0f,
                .ki = 0.1f,
                .kd = 0.0f,
                .deadband = 1,
                .maxout = VALUE_COMPARE,
                // .Improve = PID_T_Intergral | PID_I_limit | PID_D_On_Measurement | PID_D_Filter | PID_OutputFilter,
                .Improve = 0b00000000,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 500.0f,  
            }
        },
        .setting = {
            .pwm_config = {
                .htim = &htim3,
                .channel1 = TIM_CHANNEL_1,
                .channel2 = TIM_CHANNEL_2,
            },
            .flag_motor_reverse = MOTOR_DIR_NORMAL,
            .flag_feedback_reverse = FEEDBACK_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
            .motor_offset = 28.0f,

            .ptr_angle = &angle_feedforward_l,
            .ptr_speed = NULL,
        }
    };
    motor_l = Motor_Init(&motorConfig);

    motorConfig.setting.pwm_config.htim = &htim3;
    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_3;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_4;
    motorConfig.setting.flag_motor_reverse = MOTOR_DIR_REVERSE;
    motorConfig.setting.flag_feedback_reverse = FEEDBACK_DIR_REVERSE;
    motorConfig.setting.motor_offset = 216.0f;
    motorConfig.setting.ptr_angle = &angle_feedforward_r;
    motorConfig.setting.ptr_speed = NULL;
    motor_r = Motor_Init(&motorConfig);//正面

    
}


static void RemoteControl()
{
    if (sw_is_down(rc_elrs->D)){
        butterfly_mode = BUTTERFLY_MODE_STOP;
        return;
    }

    if (sw_is_up(rc_elrs->A)){
        butterfly_mode = BUTTERFLY_MODE_POSITION;
        angle_l = (rc_elrs->Left_Y - 50) * 180/50;
        angle_r = (rc_elrs->Right_Y - 50) * 180/50;
    }
    else if (sw_is_down(rc_elrs->A)){
        butterfly_mode = BUTTERFLY_MODE_FLY;
        angle_l = Al * arm_cos_f32(w * time) + bl;
        angle_r = Ar * arm_cos_f32(w * time) + br;
    }
}


static void MotorControl()
{
    //默认使能
    MotorEnable(motor_l);
    MotorEnable(motor_r);

    //默认角度闭环控制
    MotorChangeLoop(motor_l, ANGLE_LOOP);
    MotorChangeLoop(motor_r, ANGLE_LOOP);
    
    switch (butterfly_mode)
    {
        case BUTTERFLY_MODE_STOP:
        //急停模式
            MotorStop(motor_l);
            MotorStop(motor_r);
            break;
        case BUTTERFLY_MODE_POSITION:
            MotorSetRef(motor_l, angle_l);
            MotorSetRef(motor_r, angle_r);
            break;
        case BUTTERFLY_MODE_FLY:
            MotorSetRef(motor_l, angle_l);
            MotorSetRef(motor_r, angle_r);
    }
}

void Adc_Cal()
{
    static float voltage;
    static float temperature;
    voltage = ptr_adc[RANK4] * 3.3f / 4096.0f;
    temperature = ptr_adc[RANK3] * 3.3f / 4096.0f;

    angle_feedforward_l = ptr_adc[RANK1] * 360.0f / 4096.0f / 3.3f * voltage;
    angle_feedforward_r = ptr_adc[RANK2] * 360.0f / 4096.0f / 3.3f * voltage;
}


void Butterfly_Task()
{
    Adc_Cal();
    RemoteControl();
    // MotorControl();
}