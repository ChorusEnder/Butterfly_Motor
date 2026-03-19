#include "as5600.h"

#define REGISTER_CNT 4//需要配置的寄存器个数

//各寄存器地址
#define DEVICE_CONFIG_1 0x00
#define DEVICE_CONFIG_2 0x01
#define SENSOR_CONFIG_1 0x02
#define SENSOR_CONFIG_2 0x03


#define ALPHA 1.0f

float AS5600_GetAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t rawData_AS5600[2] = {0};

    // 直接从寄存器地址 0x0C 开始读两个字节（高字节 + 低字节）
    if (HAL_I2C_Mem_Read(hi2c,
                         AS5600_I2C_ADDR,
                         AS5600_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         rawData_AS5600,
                         2,
                         HAL_MAX_DELAY) != HAL_OK) {
        return -1.0f;  // 读取失败
    }

    // 合并高低字节
    uint16_t rawAngle = ((uint16_t)rawData_AS5600[0] << 8) | rawData_AS5600[1];
    rawAngle &= 0x0FFF;   // 12 位有效

    // 转换为角度 (0 ~ 360°)
    return (rawAngle * 360.0f) / 4096.0f;
}



float TMAG5273_GetAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[4];
    uint16_t code;
    float angle;
    static float previous_angle;
    float filtered_angle;

    if (HAL_I2C_Mem_Read(hi2c,
                         TMAG5273_I2C_ADDR,
                         TMAG5273_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         2,
                         HAL_MAX_DELAY) != HAL_OK) {
        return -1.0f; 
    }

    code = ((uint16_t)(data[0] << 8)) | data[1];//合并字节
    angle = (float)(code >> 4) + (float)(code & 0x000F) / 16.0f;

    // 计算滤波后的角度
    filtered_angle = ALPHA * angle + (1 - ALPHA) * previous_angle;
    previous_angle = filtered_angle;  // 更新上一个角度

    return filtered_angle;
}

void TMAG5273_ReadReg(I2C_HandleTypeDef *hi2c,uint16_t *reg_add, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c2,
                     TMAG5273_I2C_ADDR,
                     *reg_add,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     2,
                     HAL_MAX_DELAY);
}

void TMAG5273_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t *reg_add, uint8_t *data)
{
    HAL_I2C_Mem_Write(hi2c,
                      TMAG5273_I2C_ADDR,
                      *reg_add,
                      I2C_MEMADD_SIZE_8BIT,
                      data,
                      1,
                      HAL_MAX_DELAY);
}

void TMAG5273_Init(I2C_HandleTypeDef *hi2c)
{
    //配置数组
    uint16_t reg_add_w[REGISTER_CNT] = {DEVICE_CONFIG_1, DEVICE_CONFIG_2, SENSOR_CONFIG_1, SENSOR_CONFIG_2};
    uint8_t data_w[REGISTER_CNT] = {    0b00001000 ,     0b00000010,      0b00110000,      0b00000100};

    for(int i = 0; i < REGISTER_CNT; i++)
    {
        TMAG5273_WriteReg(hi2c, &reg_add_w[i], &data_w[i]);
    }
}