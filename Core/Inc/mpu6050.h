#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

typedef enum
{
    MPU6050_OK = 0,
    MPU6050_ERROR = 1
} MPU6050_Status;

typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} MPU6050_Measurement;

typedef struct {
    I2C_HandleTypeDef *i2c_handle;

    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;

    float accel_sensitivity_x;
    float accel_sensitivity_y;
    float accel_sensitivity_z;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    float gyro_sensitivity;
} MPU6050_Handle;

MPU6050_Status MPU6050_Init(
    MPU6050_Handle *imu,
    I2C_HandleTypeDef *i2c_handle
);

MPU6050_Status MPU6050_ReadAll(
    MPU6050_Handle *imu,
    MPU6050_Measurement *measurement
);

#endif
