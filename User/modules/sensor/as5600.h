#ifndef AS5600_H
#define AS5600_H

#include "i2c.h"

#define AS5600_I2C_ADDR        (0x36 << 1)  // AS5600 I2C 地址，左移一位以适应 HAL 库
#define AS5600_RAW_ANGLE_REG   0x0C          // RAW ANGLE 高字节寄存器
#define AS5600_ANGLE_REG       0x0E          // ANGLE 高字节寄存器地址

#define AS5600_CONF_H_REG      0x07          // CONF 高字节寄存器地址
#define AS5600_CONF_L_REG      0x08          // CONF 低字节寄存器地址
#define AS5600_CONF_OUTS_MASK  0x30          // OUTS(1:0) 位于 CONF 低字节 bit5:4


extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);
float AS5600_GetAngle(I2C_HandleTypeDef *hi2c);
void AS5600_SetMode_ADC(I2C_HandleTypeDef *hi2c);
void AS5600_SetMode_PWM(I2C_HandleTypeDef *hi2c);



#endif