#include "mpu6050.h"

#define MPU6050_I2C_ADDR              (0x68 << 1)

#define MPU6050_WHO_AM_I_REG          0x75
#define MPU6050_WHO_AM_I_VALUE        0x68

#define MPU6050_PWR_MGMT_1_REG        0x6B
#define MPU6050_ACCEL_XOUT_H_REG      0x3B

#define MPU6050_ACCEL_BIAS_X          1511.9f
#define MPU6050_ACCEL_BIAS_Y          23.3f
#define MPU6050_ACCEL_BIAS_Z         -4755.1f

#define MPU6050_GYRO_BIAS_X          -77.8f
#define MPU6050_GYRO_BIAS_Y          -183.8f
#define MPU6050_GYRO_BIAS_Z          -280.9f

#define MPU6050_ACCEL_SENSITIVITY_X   16295.7f
#define MPU6050_ACCEL_SENSITIVITY_Y   16302.4f
#define MPU6050_ACCEL_SENSITIVITY_Z   16829.5f

#define MPU6050_GYRO_SENSITIVITY      131.0f

#define MPU6050_MEASUREMENT_SIZE  14

#define MPU6050_SMPLRT_DIV_REG    0x19
#define MPU6050_CONFIG_REG        0x1A
#define MPU6050_GYRO_CONFIG_REG   0x1B
#define MPU6050_ACCEL_CONFIG_REG  0x1C

MPU6050_Status MPU6050_Init(
    MPU6050_Handle *imu,
    I2C_HandleTypeDef *i2c_handle
)
{
	imu->i2c_handle = i2c_handle;

	imu->accel_bias_x = MPU6050_ACCEL_BIAS_X;
	imu->accel_bias_y = MPU6050_ACCEL_BIAS_Y;
	imu->accel_bias_z = MPU6050_ACCEL_BIAS_Z;

	imu->accel_sensitivity_x = MPU6050_ACCEL_SENSITIVITY_X;
	imu->accel_sensitivity_y = MPU6050_ACCEL_SENSITIVITY_Y;
	imu->accel_sensitivity_z = MPU6050_ACCEL_SENSITIVITY_Z;

	imu->gyro_bias_x = MPU6050_GYRO_BIAS_X;
	imu->gyro_bias_y = MPU6050_GYRO_BIAS_Y;
	imu->gyro_bias_z = MPU6050_GYRO_BIAS_Z;

	imu->gyro_sensitivity = 131.0f;

	imu->gyro_sensitivity = MPU6050_GYRO_SENSITIVITY;

    uint8_t who_am_i = 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        100
    );

    if (status != HAL_OK || who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        return MPU6050_ERROR;
    }

    // Wake up

    uint8_t wake_value = 0x00;

    status = HAL_I2C_Mem_Write(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_PWR_MGMT_1_REG,
        I2C_MEMADD_SIZE_8BIT,
        &wake_value,
        1,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }


    // Gyro settings

    uint8_t gyro_config = 0x00;
    status = HAL_I2C_Mem_Write(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_GYRO_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &gyro_config,
        1,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Accelerometer settings

    uint8_t accel_config = 0x00;
    status = HAL_I2C_Mem_Write(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_ACCEL_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &accel_config,
        1,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // Debugging read back

    uint8_t accel_config_readback = 0;

    status = HAL_I2C_Mem_Read(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_ACCEL_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &accel_config_readback,
        1,
        100
    );

    if (status != HAL_OK || accel_config_readback != accel_config)
    {
        return MPU6050_ERROR;
    }

    // DLPF
    uint8_t dlpf_config = 0x03;
    status = HAL_I2C_Mem_Write(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &dlpf_config,
        1,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    // 9 + 1 = 10 -> 100 Hz

    uint8_t sample_rate_divider = 9;

    status = HAL_I2C_Mem_Write(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_SMPLRT_DIV_REG,
        I2C_MEMADD_SIZE_8BIT,
        &sample_rate_divider,
        1,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }


    return MPU6050_OK;
}

MPU6050_Status MPU6050_ReadAll(
    MPU6050_Handle *imu,
    MPU6050_Measurement *measurement
)
{
    uint8_t data[MPU6050_MEASUREMENT_SIZE];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        imu->i2c_handle,
        MPU6050_I2C_ADDR,
        MPU6050_ACCEL_XOUT_H_REG,
        I2C_MEMADD_SIZE_8BIT,
        data,
        MPU6050_MEASUREMENT_SIZE,
        100
    );

    if (status != HAL_OK)
    {
        return MPU6050_ERROR;
    }

    int16_t accel_x_raw = (data[0] << 8 | data[1]);
    int16_t accel_y_raw = (data[2] << 8 | data[3]);
    int16_t accel_z_raw = (data[4] << 8 | data[5]);

    measurement->accel_x_raw = accel_x_raw;
    measurement->accel_y_raw = accel_y_raw;
    measurement->accel_z_raw = accel_z_raw;

    // Omit temperature

    int16_t gyro_x_raw = (data[8] << 8 | data[9]);
    int16_t gyro_y_raw = (data[10] << 8 | data[11]);
    int16_t gyro_z_raw = (data[12] << 8 | data[13]);

    measurement->gyro_x_raw = gyro_x_raw;
    measurement->gyro_y_raw = gyro_y_raw;
    measurement->gyro_z_raw = gyro_z_raw;

    measurement->accel_x_g =
        (accel_x_raw - imu->accel_bias_x)
        / imu->accel_sensitivity_x;

    measurement->accel_y_g =
        (accel_y_raw - imu->accel_bias_y)
        / imu->accel_sensitivity_y;

    measurement->accel_z_g =
        (accel_z_raw - imu->accel_bias_z)
        / imu->accel_sensitivity_z;

    measurement->gyro_x_dps =
        ((float)measurement->gyro_x_raw - imu->gyro_bias_x)
        / imu->gyro_sensitivity;

    measurement->gyro_y_dps =
        ((float)measurement->gyro_y_raw - imu->gyro_bias_y)
        / imu->gyro_sensitivity;

    measurement->gyro_z_dps =
        ((float)measurement->gyro_z_raw - imu->gyro_bias_z)
        / imu->gyro_sensitivity;

    return MPU6050_OK;
}
