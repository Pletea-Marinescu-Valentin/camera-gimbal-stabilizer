#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* MPU6050 Device Address */
#define MPU6050_ADDR           0xD0  // 0x68 << 1 (for HAL I2C functions)

/* MPU6050 Registers */
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_GYRO_XOUT_H    0x43

/* Function Prototypes */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_IsConnected(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);
HAL_StatusTypeDef MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c, float* temp);

#endif /* MPU6050_H */
