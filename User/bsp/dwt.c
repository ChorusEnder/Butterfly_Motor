#include "dwt.h"

static DWT_Time_s DWT_Time;//时间轴结构体实例
static uint32_t CPU_Freq_Hz, CPU_Freq_us, CPU_Freq_ms;//除数,将周期数转换为对应的的时间
static uint32_t CYCCNT_RoundCount;//记录周期计数器圈数
static uint32_t CYCCNT_Last;//上次读取的周期计数器值
static uint64_t CYCCNT64;//周期计数器总计数

/**
 * @brief 私有函数,更新周期计数器的值,并处理溢出
 */
static void DWT_CNT_Update(void)
{
    //锁变量,在调用期间处于上锁状态,防止在函数重入导致数据错误
    static volatile uint8_t locker = 0;

    if (!locker){
        locker = 1;//上锁
        volatile uint32_t CYCCNT_Now = DWT->CYCCNT;//读取周期寄存器的值
        if (CYCCNT_Now < CYCCNT_Last) {
            CYCCNT_RoundCount++;
        }
        CYCCNT_Last = CYCCNT_Now;
        locker = 0;//解锁
    }
}

void DWT_Init(uint32_t CPU_MHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_Freq_Hz = CPU_MHz * 1000000;
    CPU_Freq_ms = CPU_MHz * 1000;
    CPU_Freq_us = CPU_MHz;
    CYCCNT_RoundCount = 0;

    DWT_CNT_Update();
}

float DWT_GetDeltaT_s(uint32_t *cnt_last)
{
    
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt= (uint32_t)(cnt_now - *cnt_last) / ((float)CPU_Freq_Hz);
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64_s(uint32_t *cnt_last)
{
    
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt= (uint32_t)(cnt_now - *cnt_last) / ((double)CPU_Freq_Hz);
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief 私有函数,更新时间结构体DWT_Time的值
 */
static void DWT_SysTime_Update(void)
{
    //volatile修饰防止优化,使其每次访问都要真的去读或写内存地址
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_Temp1, CNT_Temp2, CNT_Temp3;//临时变量,用于计算各时间轴

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RoundCount * (uint64_t)(UINT32_MAX+1) + (uint64_t)cnt_now;
    CNT_Temp1 = CYCCNT64 / CPU_Freq_Hz;
    CNT_Temp2 = CYCCNT64 - CNT_Temp1 * CPU_Freq_Hz;
    DWT_Time.s = CNT_Temp1;
    DWT_Time.ms = CNT_Temp2 / CPU_Freq_ms;
    CNT_Temp3 = CNT_Temp2 - DWT_Time.ms * CPU_Freq_ms;
    DWT_Time.us = CNT_Temp3 / CPU_Freq_us;
}

float DWT_GetTimeLine_s()
{
    DWT_SysTime_Update();

    float timeline_32f = DWT_Time.s + DWT_Time.ms * 0.001f + DWT_Time.us * 0.000001f;

    return timeline_32f;
}

float DWT_GetTimeLine_ms()
{
    DWT_SysTime_Update();

    float timeline_ms = DWT_Time.s * 1000 + DWT_Time.ms + DWT_Time.us * 0.001f;

    return timeline_ms;
}

uint64_t DWT_GetTimeLine_us()
{
    DWT_SysTime_Update();

    uint64_t timeline_us = DWT_Time.s * 1000000 + DWT_Time.ms * 1000 + DWT_Time.us;

    return timeline_us;
}
