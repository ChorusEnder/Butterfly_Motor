#pragma once

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "dwt.h"
#include "daemon.h"
#include "butterfly.h"
#include "motor.h"


osThreadId motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId butterflyTaskHandle;
const osThreadAttr_t butterflyTask_attributes = {
  .name = "butterflyTask",
  .stack_size = 128 * 6,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId daemonTaskHandle;
const osThreadAttr_t daemonTask_attributes = {
  .name = "daemonTask",
  .stack_size = 128 * 2,
  .priority = (osPriority_t) osPriorityNormal1,
};


void motorTASK(void *argument);
void butterflyTASK(void *argument);
void daemonTASK(void *argument);

void OSTask_Init(void)
{
    motorTaskHandle = osThreadNew(motorTASK, NULL, &motorTask_attributes);
    butterflyTaskHandle = osThreadNew(butterflyTASK, NULL, &butterflyTask_attributes);
    daemonTaskHandle = osThreadNew(daemonTASK, NULL, &daemonTask_attributes);
}

void motorTASK(void *argument)
{

    /* Infinite loop */
    for(;;)
    {
        //频率过快似乎会导致iic通信失败
        MotorTask();//任务运行时间约0.5ms
        osDelay(2);

    }
}

void butterflyTASK(void *argument)
{
    
    /* Infinite loop */
    for(;;)
    {
        Butterfly_Task();
        osDelay(2);
    }
}

void daemonTASK(void *argument)
{
    /* Infinite loop */
    for(;;)
    {
        DaemonTask();
        osDelay(10);
    }
}

