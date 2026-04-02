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
#include "math.h"

#define ADC_BANDWIDTH_1 4095.0f//电机左边带宽,单位:度
#define ADC_BANDWIDTH_2 4095.0f//电机右边带宽,单位:
#define ADC_OFFSET_1 1736.0f
#define ADC_OFFSET_2 2888.0f
#define ADC_MIN_1 (ADC_OFFSET_1 - 1000.0f)
#define ADC_MAX_1 (ADC_OFFSET_1 + 1000.0f)
#define ADC_MIN_2 (ADC_OFFSET_2 - 1000.0f)
#define ADC_MAX_2 (ADC_OFFSET_2 + 1000.0f)


static float vrefint;
static float temperature;

static butterfly_mode_e butterfly_mode;
static Motor_Instance_s* motor_l;
static Motor_Instance_s* motor_r;
static ELRS_Data *rc_elrs;
static uint16_t *ptr_adc;
static float angle_feedback_1;
static float angle_feedback_2;

static float angle_feedforward_1;
static float angle_feedforward_2;

/*-------------------以下是Asin(wt)+B有关的参数---------*/
static float angle_l;
static float angle_r;
static float time;
static float w = 16;//角速度,单位:rad/s
static float Al = 50;
static float bl = -10;
static float Ar = 50;
static float br = -10;
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
            .loop_type = ANGLE_LOOP,
            .pid_ref = 0.0f,
            .angle_pid = {
                .kp = 10.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .deadband = 1.0f,
                .maxout = 900,
                .Improve = PID_T_Intergral | PID_I_limit | PID_OutputFilter | PID_Changing_I,
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
                .channel1 = TIM_CHANNEL_3,
                .channel2 = TIM_CHANNEL_4,
            },
            .flag_motor_reverse = MOTOR_DIR_REVERSE,
            .flag_feedback_reverse = FEEDBACK_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
            .motor_offset = 0.0f,

            .ptr_angle = &angle_feedback_1,
            .ptr_speed = NULL,
        }
    };
    motor_r = Motor_Init(&motorConfig);

    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_1;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_2;
    motorConfig.setting.flag_motor_reverse = MOTOR_DIR_REVERSE;
    motorConfig.setting.flag_feedback_reverse = FEEDBACK_DIR_NORMAL;
    motorConfig.setting.motor_offset = 0.0f;
    motorConfig.setting.ptr_angle = &angle_feedback_2;
    motorConfig.setting.ptr_speed = NULL;
    motor_l = Motor_Init(&motorConfig);

    
}


static void RemoteControl()
{
    butterfly_mode = BUTTERFLY_MODE_POSITION;
    
    if (rc_elrs->rc_state == RC_OFFLINE) {
        butterfly_mode = BUTTERFLY_MODE_STOP;
        return;
    }
    if (sw_is_down(rc_elrs->D)){
        butterfly_mode = BUTTERFLY_MODE_STOP;
        return;
    }

    if (sw_is_up(rc_elrs->A)){
        butterfly_mode = BUTTERFLY_MODE_POSITION;
        angle_l = (rc_elrs->Left_Y - 50);
        angle_r = (rc_elrs->Left_Y - 50);
    }
    else if (sw_is_down(rc_elrs->A)){
        butterfly_mode = BUTTERFLY_MODE_FLY;
        angle_l = cosf(time * w) * Al + bl;
        angle_r = cosf(time * w) * Ar + br;
    }
}


static void MotorControl()
{
    //默认使能
    MotorEnable(motor_l);
    MotorEnable(motor_r);

    //默认角度闭环控制
    // MotorChangeLoop(motor_l, ANGLE_LOOP);
    // MotorChangeLoop(motor_r, ANGLE_LOOP);

    //前馈计算
    angle_feedforward_1 = 100 *cosf(angle_feedback_1 * ANG_TO_RAD);
    angle_feedforward_2 = 100 *cosf(angle_feedback_2 * ANG_TO_RAD);
    MotorSetFeedforward(motor_l, angle_feedforward_1);
    MotorSetFeedforward(motor_r, angle_feedforward_2);

    //限幅
    if (angle_l > 40.0f) angle_l = 40.0f;
    if (angle_l < -80.0f) angle_l = -80.0f;
    if (angle_r > 40.0f) angle_r = 40.0f;
    if (angle_r < -80.0f) angle_r = -80.0f;

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
    uint16_t temperature_value;
    float resistance;
    
    //电压参考源
    vrefint = ADC_BANDWIDTH / ptr_adc[RANK4] * ADC_VREFINT_TYPE;

    //温度计算
    temperature_value = ptr_adc[RANK3];
    resistance = temperature_value / (ADC_BANDWIDTH - temperature_value) * 10000.0f;//10k分压电阻
    static float B = 3950.0f;
    static float R2 = 10000.0f;
    static float T2 = 25.0f;
    temperature =  (1.0 / ((1.0 / B) * log(resistance / R2) + (1.0 / (T2 + 273.15))) - 273.15);
        
    /*-------------------------------------反馈角度计算---------------------------*/

    float adc_raw_1 = ptr_adc[RANK1];
    float adc_raw_2 = ptr_adc[RANK2];
    float adc_1 = adc_raw_1;
    float adc_2 = adc_raw_2;


    angle_feedback_1 = (adc_1 - ADC_OFFSET_1) / ADC_BANDWIDTH_1 * 270.f;
    angle_feedback_2 = (adc_2 - ADC_OFFSET_2) / ADC_BANDWIDTH_2 * 270.f;
}


void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();
    Adc_Cal();
    RemoteControl();
    MotorControl();
}