#ifndef AS5600_H
#define AS5600_H

#include "i2c.h"

#define AS5600_I2C_ADDR        (0x36 << 1)  // AS5600 I2C 地址，左移一位以适应 HAL 库
#define AS5600_RAW_ANGLE_REG   0x0C          // RAW ANGLE 高字节寄存器

#define TMAG5273_I2C_ADDR        (0x35 << 1)  // TMAG5273 I2C 地址，左移一位以适应 HAL 库
#define TMAG5273_RAW_ANGLE_REG   0x19          // RAW ANGLE 高字节寄存器

extern I2C_HandleTypeDef hi2c1;

float AS5600_GetAngle(I2C_HandleTypeDef *hi2c);
float TMAG5273_GetAngle(I2C_HandleTypeDef *hi2c);

void TMAG5273_ReadReg(I2C_HandleTypeDef *hi2c,uint16_t *reg_add, uint8_t *data);
void TMAG5273_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t *reg_add, uint8_t *data);
void TMAG5273_Init(I2C_HandleTypeDef *hi2c);

#endif