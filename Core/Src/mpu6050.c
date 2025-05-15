/* mpu6050.c */
#include "mpu6050.h"
#include <stdio.h>

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t data;

    // Reset device
    data = 0x80;  // Set reset bit
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    HAL_Delay(100);  // Wait for reset to complete

    // Wake up device
    data = 0x00;  // Clear sleep bit
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set sample rate
    data = 0x07;  // 1kHz / (1 + 7) = 125Hz
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set accelerometer configuration (±2g range)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set gyroscope configuration (±250°/s range)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    return status;
}

HAL_StatusTypeDef MPU6050_IsConnected(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status;

    // Read WHO_AM_I register
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &who_am_i, 1, HAL_MAX_DELAY);

    if (status == HAL_OK && who_am_i == 0x68) {
        printf("MPU6050 connected successfully! WHO_AM_I = 0x%02X\r\n", who_am_i);
        return HAL_OK;
    } else {
        printf("MPU6050 connection failed! WHO_AM_I = 0x%02X, status = %d\r\n", who_am_i, status);
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t* accel_x, int16_t* accel_y, int16_t* accel_z)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    // Read 6 bytes of data starting from ACCEL_XOUT_H register
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        // Combine the bytes to form 16-bit signed integers (big-endian)
        *accel_x = (int16_t)(data[0] << 8 | data[1]);
        *accel_y = (int16_t)(data[2] << 8 | data[3]);
        *accel_z = (int16_t)(data[4] << 8 | data[5]);
    }

    return status;
}

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    // Read 6 bytes of data starting from GYRO_XOUT_H register
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        *gyro_x = (int16_t)(data[0] << 8 | data[1]);
        *gyro_y = (int16_t)(data[2] << 8 | data[3]);
        *gyro_z = (int16_t)(data[4] << 8 | data[5]);
    }

    return status;
}

HAL_StatusTypeDef MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c, float* temp)
{
    uint8_t data[2];
    int16_t raw_temp;
    HAL_StatusTypeDef status;

    // Read 2 bytes of data starting from TEMP_OUT_H register
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        raw_temp = (int16_t)(data[0] << 8 | data[1]);
        // Convert to temperature in °C (formula from datasheet)
        *temp = (float)raw_temp / 340.0f + 36.53f;
    }

    return status;
}
