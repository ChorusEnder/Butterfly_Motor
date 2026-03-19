#include "tim.h"


typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} SlaveMotor_PWM_Config_s;

typedef struct {
    SlaveMotor_PWM_Config_s pwm_config;
    
}