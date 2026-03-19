#pragma once
#include "tim.h"
#include <stdint.h>


// 时间结构体,分为秒,毫秒,微秒
typedef struct {
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} Time_s;

/**
 * @brief DWT初始化函数
 * @param CPU_MHz CPU主频,单位MHz
 * @note 此函数使用的是基本定时器,所以在cubemx中配置的定时器预分频系数为0,自动重装载寄存器为最大值
 * @note 并且需选定自动重装载值范围为32位(函数逻辑基于32位计数器),否则可能会出现计数器溢出过快导致时间计算错误的情况,后期考虑兼容16位计数器
 */
void Timer_Init(TIM_HandleTypeDef *TIMx, uint32_t CPU_MHz);

/**
 * @brief 获取系统时间线,单位秒
 * @return 系统时间线,单位秒
 */
float Timer_GetTime_s();

/**
 * @brief 获取系统时间线,单位毫秒
 * @return 系统时间线,单位毫秒
 */
float Timer_GetTime_ms();

/**
 * @brief 获取系统时间线,单位微秒
 * @return uint64_t 系统时间线,单位微秒
 */
uint64_t Timer_GetTime_us();

/**
 * @brief 获取两次调用的时间差,单位秒
 * @param cnt_last 传入一个uin32_t类型的变量地址即可
 * @return 时间差,单位秒
 */
float Timer_GetDeltaT_s(uint32_t *cnt_last);

/**
 * @brief 获取两次调用的时间差,单位秒,使用double类型计算,精度更高
 * @param cnt_last 传入一个uin32_t类型的变量地址即可
 * @return 时间差,单位秒
 */
double Timer_GetDeltaT64_s(uint32_t *cnt_last);