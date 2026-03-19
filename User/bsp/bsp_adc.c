#include "bsp_adc.h"

static uint16_t adc_value[ADC_CH_CNT];

uint16_t *BSP_ADC_Init(ADC_HandleTypeDef *hadc)
{
    HAL_ADCEx_Calibration_Start(hadc);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_value, ADC_CH_CNT);
    
    return adc_value;
}
