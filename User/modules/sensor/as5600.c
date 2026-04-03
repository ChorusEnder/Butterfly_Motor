#include "as5600.h"

HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((hi2c == NULL) || (data == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(hi2c,
                            AS5600_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            HAL_MAX_DELAY);
}


float AS5600_GetAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t rawData_AS5600[2] = {0};

    // 直接从寄存器地址 0x0E 开始读两个字节（高字节 + 低字节）
    if (AS5600_ReadReg(hi2c,
                       AS5600_ANGLE_REG,
                       rawData_AS5600,
                       2) != HAL_OK) {
        return -1.0f;  // 读取失败
    }

    // 合并高低字节
    uint16_t rawAngle = ((uint16_t)rawData_AS5600[0] << 8) | rawData_AS5600[1];
    rawAngle &= 0x0FFF;   // 12 位有效

    // 转换为角度 (0 ~ 360°)
    return (rawAngle * 360.0f) / 4096.0f;
}

void AS5600_SetMode_ADC(I2C_HandleTypeDef *hi2c)
{
    uint8_t conf[2] = {0};

    // CONF 寄存器跨 0x07/0x08，OUTS 在低字节 bit5:4
    if (AS5600_ReadReg(hi2c,
                       AS5600_CONF_H_REG,
                       conf,
                       2) != HAL_OK) {
        return;
    }

    // OUTS=01: 模拟输出缩放(10%~90% of VDD)
    conf[1] = (uint8_t)(conf[1] & (uint8_t)(~AS5600_CONF_OUTS_MASK));

    (void)HAL_I2C_Mem_Write(hi2c,
                            AS5600_I2C_ADDR,
                            AS5600_CONF_H_REG,
                            I2C_MEMADD_SIZE_8BIT,
                            conf,
                            2,
                            HAL_MAX_DELAY);
}

void AS5600_SetMode_PWM(I2C_HandleTypeDef *hi2c)
{
    uint8_t conf[2] = {0};

    // CONF 寄存器跨 0x07/0x08，OUTS 在低字节 bit5:4
    if (AS5600_ReadReg(hi2c,
                       AS5600_CONF_H_REG,
                       conf,
                       2) != HAL_OK) {
        return;
    }

    // OUTS=10: PWM 输出
    conf[1] = (uint8_t)((conf[1] & (uint8_t)(~AS5600_CONF_OUTS_MASK)) | 0x20U);

    (void)HAL_I2C_Mem_Write(hi2c,
                            AS5600_I2C_ADDR,
                            AS5600_CONF_H_REG,
                            I2C_MEMADD_SIZE_8BIT,
                            conf,
                            2,
                            HAL_MAX_DELAY);
}

