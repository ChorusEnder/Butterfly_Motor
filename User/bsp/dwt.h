#ifndef DWT_H
#define DWT_H

#include "main.h"

// 时间结构体,分为秒,毫秒,微秒
typedef struct {
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} DWT_Time_s;

/**
 * @brief DWT初始化函数
 * @param CPU_MHz CPU主频,单位MHz
 */
void DWT_Init(uint32_t CPU_MHz);

/**
 * @brief 获取系统时间线,单位秒
 * @return 系统时间线,单位秒
 */
float DWT_GetTimeLine_s();

/**
 * @brief 获取系统时间线,单位毫秒
 * @return 系统时间线,单位毫秒
 */
float DWT_GetTimeLine_ms();

/**
 * @brief 获取系统时间线,单位微秒
 * @return uint64_t 系统时间线,单位微秒
 */
uint64_t DWT_GetTimeLine_us();

/**
 * @brief 获取两次调用的时间差,单位秒
 * @param cnt_last 传入一个uin32_t类型的变量地址即可
 * @return 时间差,单位秒
 */
float DWT_GetDeltaT_s(uint32_t *cnt_last);

/**
 * @brief 获取两次调用的时间差,单位秒,使用double类型计算,精度更高
 * @param cnt_last 传入一个uin32_t类型的变量地址即可
 * @return 时间差,单位秒
 */
double DWT_GetDeltaT64_s(uint32_t *cnt_last);


#endif