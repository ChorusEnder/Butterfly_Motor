#include "remote_fs.h"
#include "bsp_uart.h"
#include "daemon.h"
#include <stdlib.h>

/**-------------I-bus define-------------- */
#define IBUS_HEADER 0x20 // IBUS数据包头
#define IBUS_DATA_LEN 32 // 32字节数据包
#define IBUS_OFFSET 1500 //遥控器通道偏置
#define IBUS_SW_DIV 500   //利用C语言中`/`的性质匹配开关状态
/**---------------------------------------- */

/**-------------Sbus define-------------- */
#define RC_FS_RXBUFF_SIZE 25 // 遥控器接收的buffer大小
#define SBUS_HEADER 0x0F
#define SBUS_MID 1024
#define SBUS_SW_DIV 500//直接`/`通道值得到开关状态
/*-----------------------------------------------*/

#define rocker_deadband(x) ((abs(x) < 10) ? 0 : x) // 摇杆死区处理,处理垃圾富斯遥感零偏

static RC_Fs_Ctrl_s rc_data; // 遥控器数据
static UART_Instance *rc_uart_instance;
static Daemon_Instance *rc_daemon_instance;

//函数原型
static void Decode_Ibus();
static void Decode_Sbus();
static void Ch_to_Ctrl_Fs();

//接收回调函数
static void Remote_fs_RxCallback()
{
    DaemonReload(rc_daemon_instance); // 先喂狗

    if (rc_data.type == RC_TYPE_SBUS)
        Decode_Sbus();//协议解析
    else if (rc_data.type == RC_TYPE_IBUS)
        Decode_Ibus();//协议解析

    Ch_to_Ctrl_Fs();//通道值映射到遥控器控的实际按键控制
}

static void Ch_to_Ctrl_Fs()
{
    int16_t *ch = rc_data.channel;

    //摇杆部分基本不用改
    rc_data.rocker_r_ = ch[0];
    rc_data.rocker_r1 = ch[1];
    rc_data.rocker_l1 = ch[2];
    rc_data.rocker_l_ = ch[3];
    rc_data.rocker_l1 = rocker_deadband(rc_data.rocker_l1);
    rc_data.rocker_l_ = rocker_deadband(rc_data.rocker_l_);
    rc_data.rocker_r1 = rocker_deadband(rc_data.rocker_r1);
    rc_data.rocker_r_ = rocker_deadband(rc_data.rocker_r_);

    //通道匹配遥控器设置
    rc_data.swa = ch[4] / SBUS_SW_DIV;
    rc_data.swb = ch[5] / SBUS_SW_DIV;
    rc_data.swc = ch[6] / SBUS_SW_DIV;
    rc_data.swd = ch[7] / SBUS_SW_DIV;

    rc_data.vra = ch[8];
    rc_data.vrb = ch[9];
}

static void Decode_Sbus()
{
    if (rc_uart_instance->rx_buffer[0] != SBUS_HEADER) return;

    int16_t *ch = rc_data.channel;
    const uint8_t * buf = rc_uart_instance->rx_buffer;

    ch[0]  = ((buf[1]     | buf[2]<<8) & 0x07FF) - SBUS_MID;
    ch[1]  = ((buf[2]>>3  | buf[3]<<5) & 0x07FF) - SBUS_MID;
    ch[2]  = ((buf[3]>>6  | buf[4]<<2 | buf[5]<<10) & 0x07FF) - SBUS_MID;
    ch[3]  = ((buf[5]>>1  | buf[6]<<7) & 0x07FF) - SBUS_MID;
    ch[4]  = ((buf[6]>>4  | buf[7]<<4) & 0x07FF) - SBUS_MID;
    ch[5]  = ((buf[7]>>7  | buf[8]<<1 | buf[9]<<9) & 0x07FF) - SBUS_MID;
    ch[6]  = ((buf[9]>>2  | buf[10]<<6) & 0x07FF) - SBUS_MID;
    ch[7]  = ((buf[10]>>5 | buf[11]<<3) & 0x07FF) - SBUS_MID;
    ch[8]  = ((buf[12]    | buf[13]<<8) & 0x07FF) - SBUS_MID;
    ch[9]  = ((buf[13]>>3 | buf[14]<<5) & 0x07FF) - SBUS_MID;
    ch[10] = ((buf[14]>>6 | buf[15]<<2 | buf[16]<<10) & 0x07FF) - SBUS_MID;
    ch[11] = ((buf[16]>>1 | buf[17]<<7) & 0x07FF) - SBUS_MID;
    ch[12] = ((buf[17]>>4 | buf[18]<<4) & 0x07FF) - SBUS_MID;
    ch[13] = ((buf[18]>>7 | buf[19]<<1 | buf[20]<<9) & 0x07FF) - SBUS_MID;
    ch[14] = ((buf[20]>>2 | buf[21]<<6) & 0x07FF) - SBUS_MID;
    ch[15] = ((buf[21]>>5 | buf[22]<<3) & 0x07FF) - SBUS_MID;
}


static void RCLostCallback()
{
    memset(&rc_data, 0, sizeof(RC_Fs_Ctrl_s));
}

RC_Fs_Ctrl_s* RC_Fs_Init_Sbus(UART_HandleTypeDef *usart_handle)
{
    UART_Init_Config conf;
    conf.callback = Remote_fs_RxCallback;
    conf.huart_ptr = usart_handle;
    conf.rx_buffer_size = RC_FS_RXBUFF_SIZE;
    rc_uart_instance = UART_Init(&conf);

    rc_data.type = RC_TYPE_SBUS;

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 100,
        .init_count = 200,
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonInit(&daemon_conf);

    return &rc_data;
}





/**-------------Ibus Fuction-------------- */

// 'ibus'模块的私有函数,计算校验和
static uint16_t IBUS_Checksum(uint8_t *buf)
{
    uint16_t chksum = 0xFFFF;
    for (int i = 0; i < IBUS_DATA_LEN - 2; i++)
        chksum -= buf[i];
    return chksum;
}

/**
 * @brief 'ibus'模块的私有函数,解码
 */
static void Decode_Ibus()
{
    uint8_t *rx_buff = rc_uart_instance->rx_buffer;//接收缓存
    int16_t *ch = rc_data.channel;

    //帧头匹配
    if (rx_buff[0] != IBUS_HEADER) return;

    //校验和
    uint16_t sum = IBUS_Checksum(rx_buff);
    uint16_t rxsum = rx_buff[30] | (rx_buff[31] << 8);
    if (sum != rxsum) return;


    //解析通道数据
    for (int i = 0; i < 14; i++)
        ch[i] = (rx_buff[2 + i * 2] | (rx_buff[3 + i * 2] << 8)) - IBUS_OFFSET;
}

/**
 * @brief 'ibus'模块的公有函数,解码
 * @param 
 * 
 */
RC_Fs_Ctrl_s *RC_Fs_Init_Ibus(UART_HandleTypeDef *huart)
{
    UART_Init_Config config;
    config.rx_buffer_size = IBUS_DATA_LEN;
    config.huart_ptr = huart;
    config.callback = Remote_fs_RxCallback;
    rc_uart_instance = UART_Init(&config);

    rc_data.type = RC_TYPE_IBUS;

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, 
        .init_count = 20,
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonInit(&daemon_conf);

    return &rc_data;
}


/*-----------------------------------------------------*/