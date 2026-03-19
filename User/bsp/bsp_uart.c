#include "bsp_uart.h"

#define UART_INSTANCE_COUNT 2

static UART_Instance *uart_instance[UART_INSTANCE_COUNT];
static uint8_t idx;
uint32_t uart_rx_cnt = 0;

UART_Instance *UART_Init(UART_Init_Config *config)
{
    UART_Instance *instance = (UART_Instance*)malloc(sizeof(UART_Instance));
    memset(instance, 0, sizeof(UART_Instance));
    instance->huart = config->huart_ptr;
    instance->rx_buffer_size = config->rx_buffer_size;
    instance->callback = config->callback;

    uart_instance[idx++] = instance;

    //开启DMA接收,并禁止半传输中断
    HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, instance->rx_buffer, instance->rx_buffer_size);
    __HAL_DMA_DISABLE_IT(instance->huart->hdmarx, DMA_IT_HT);

    return instance;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; i++) {
        if (uart_instance[i]->huart == huart){

            if (uart_instance[i]->callback != NULL) {
                uart_instance[i]->callback(); //调用回调函数
                uart_rx_cnt++;
                memset(uart_instance[i]->rx_buffer, 0, Size);//清空接收缓冲区
            }
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instance[i]->huart, uart_instance[i]->rx_buffer, uart_instance[i]->rx_buffer_size);
            __HAL_DMA_DISABLE_IT(uart_instance[i]->huart->hdmarx, DMA_IT_HT);
        }
    }
}
