# pragma once

#include "main.h"

typedef enum {
    BUTTERFLY_MODE_STOP,//急停模式
    BUTTERFLY_MODE_POSITION,//位置模式
    BUTTERFLY_MODE_FLY,//飞行模式
} butterfly_mode_e;


void Butterfly_Init();
void Butterfly_Task();

