#include "bsp_timer.h"

static Time_s Timeline;//时间轴结构体实例
static TIM_HandleTypeDef *htim;//用于定时的定时器
static uint32_t CPU_Freq_Hz, CPU_Freq_us, CPU_Freq_ms;//除数,将周期数转换为对应的的时间
static uint32_t CNT_TIM_Last;//上次读取的计数器值
static uint32_t CNT_Overflow;//总的溢出次数
static uint64_t CNT_64;//计数器总计数


//更新计数器的溢出次数
static void Overflow_Update()
{
    //锁变量,在调用期间处于上锁状态,防止在函数重入导致数据错误
    static volatile uint8_t locker = 0;

    if (!locker){
        locker = 1;//上锁
        volatile uint32_t CNT_TIM_Now = __HAL_TIM_GET_COUNTER(htim);//读取计数器寄存器的值
        if (CNT_TIM_Now < CNT_TIM_Last) {
            CNT_Overflow++;
        }
        CNT_TIM_Last = CNT_TIM_Now;
        locker = 0;//解锁
    }
}


void Timer_Init(TIM_HandleTypeDef *htimx, uint32_t CPU_MHz)    
{
    htim = htimx;
    HAL_TIM_Base_Start(htimx);//启动定时器

    CPU_Freq_Hz = CPU_MHz * 1000000;
    CPU_Freq_us = CPU_Freq_Hz / 1000000;
    CPU_Freq_ms = CPU_Freq_Hz / 1000;

    Overflow_Update();
}


//更新时间结构体
static void Time_Update()
{
    volatile uint32_t cnt_now = __HAL_TIM_GET_COUNTER(htim);//读取当前计数器的值 

    Overflow_Update();

    CNT_64 = ((uint64_t)CNT_Overflow << 32) | cnt_now;
    Timeline.s = CNT_64 / CPU_Freq_Hz;
    Timeline.ms = (CNT_64 % CPU_Freq_Hz) / CPU_Freq_ms;
    Timeline.us = (CNT_64 % CPU_Freq_ms) / CPU_Freq_us;
} 

float Timer_GetTime_s()
{
    Time_Update();
    return (float)Timeline.s + (float)Timeline.ms / 1000.0f + (float)Timeline.us / 1000000.0f;
}


float Timer_GetTime_ms()
{
    Time_Update();
    return (float)Timeline.s * 1000.0f + (float)Timeline.ms + (float)Timeline.us / 1000.0f;
}


uint64_t Timer_GetTime_us()
{
    Time_Update();
    return (uint64_t)Timeline.s * 1000000 + (uint64_t)Timeline.ms * 1000 + (uint64_t)Timeline.us;
}


float Timer_GetDeltaT_s(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = __HAL_TIM_GET_COUNTER(htim);
    //uint类型的变量相减,若结果小于0,会自动下溢,即自动加上2^32,因此无需特殊处理
    float dt = (uint32_t)(cnt_now - *cnt_last) / ((float)CPU_Freq_Hz);
    *cnt_last = cnt_now;
    return dt;
}


double Timer_GetDeltaT64_s(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = __HAL_TIM_GET_COUNTER(htim);
    uint64_t cnt_diff = (uint64_t)(cnt_now - *cnt_last);
    double dt = (double)cnt_diff / (double)CPU_Freq_Hz;
    *cnt_last = cnt_now;
    return dt;
}