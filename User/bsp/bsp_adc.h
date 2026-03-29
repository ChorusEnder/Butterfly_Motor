#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "adc.h"
#include "main.h"

#define ADC_CH_CNT 4

#define ADC_VREFINT_TYPE 1.20f//电压参考源
#define ADC_BANDWIDTH 4095.0f

typedef enum {
    RANK1 = 0,
    RANK2,
    RANK3,
    RANK4,
    RANK5,
    RANK6,
    RANK7,
    RANK8
} Rank;

uint16_t *BSP_ADC_Init(ADC_HandleTypeDef *hadc);




#endif