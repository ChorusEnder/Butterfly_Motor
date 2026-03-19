#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>

#define UART_RX_BUFFER_SIZE 64

typedef void (*uart_callback)();//回调函数,用于解析数据包

typedef struct {
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];//最大数据存储数组
    uint8_t rx_buffer_size;//实际包大小
    UART_HandleTypeDef *huart;
    uart_callback callback;
    uint8_t call_size;
} UART_Instance;

typedef struct {
    uint8_t rx_buffer_size;
    UART_HandleTypeDef *huart_ptr;
    uart_callback callback;
} UART_Init_Config;


UART_Instance *UART_Init(UART_Init_Config *config);






#endif