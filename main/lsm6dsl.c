#include <stdio.h>
#include "lsm6dsl.h"
#include "i2c.h"
#include "math.h"


static float *acc_runtime_bias_ptr;
const float accel_bias_ms2[3] = {-0.0410183726183868f, -0.0476472728044818f, 0.249395466543557f};
const float accel_calib_matrix[3][3] = 
{
    {0.994499660760511f, 0.000411617480116379f, -0.00333746338878276f},
    {0.000411617480116379f,	0.995922546944084f,	0.00184908061381518f},
    {-0.00333746338878276f,	0.00184908061381518f, 1.00341339931936f}
};


static void getCalibratedResults(lsm6dsl_t *imu);
static void parse_buffer(uint8_t *buff, lsm6dsl_t *imu);

void lsm6dsl_setup(float *acc_cal)
{
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL1_XL, ODR_1666_HZ | ACCEL_8G | 3); // 0xAF
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL8_XL, 0xE1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL2_G, ODR_1666_HZ | GYRO_1000DPS | 0);// 0X98
    vTaskDelay(20 / portTICK_PERIOD_MS); 
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL4_C, 0x02);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL6_C, 0x42);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    acc_runtime_bias_ptr = acc_cal;
}

void lsmldsl_read(lsm6dsl_t *imu)
{
    static uint8_t buff[14] = {0};
    i2c_read_data(I2C_NUM_0, LSM6DSL_ADDR, OUT_TEMP_L, buff, 14);
    parse_buffer(buff, imu);
    getCalibratedResults(imu);
}

static void parse_buffer(uint8_t *buff, lsm6dsl_t *imu)
{
    imu->temp_mC = ((int16_t)(buff[0] | buff[1] << 8) / TEMP_SENSITIVITY_MC) + 2500;
    imu->gyro_dps[X] = (int16_t)(buff[2] | buff[3] << 8) * GYRO_SENSITIVITY_DEGS;
    imu->gyro_dps[Y] = (int16_t)(buff[4] | buff[5] << 8) * GYRO_SENSITIVITY_DEGS;
    imu->gyro_dps[Z] = (int16_t)(buff[6] | buff[7] << 8) * GYRO_SENSITIVITY_DEGS;

    imu->accel_ms2[X] = (int16_t)(buff[8] | buff[9] << 8) * ACCEL_SENSITIVITY_MS2;
    imu->accel_ms2[Y] = (int16_t)(buff[10] | buff[11] << 8) * ACCEL_SENSITIVITY_MS2;
    imu->accel_ms2[Z] = (int16_t)(buff[12] | buff[13] << 8) * ACCEL_SENSITIVITY_MS2;
}

static void getCalibratedResults(lsm6dsl_t *imu)
{
    imu->gyro_dps[X] -= imu->gyro_bias_dps[X];
    imu->gyro_dps[Y] -= imu->gyro_bias_dps[Y];
    imu->gyro_dps[Z] -= imu->gyro_bias_dps[Z];

    static float temp_x = 0;
    static float temp_y = 0;
    static float temp_z = 0;

    temp_x = imu->accel_ms2[X] - accel_bias_ms2[X] - acc_runtime_bias_ptr[0];
    temp_y = imu->accel_ms2[Y] - accel_bias_ms2[Y] - acc_runtime_bias_ptr[1];
    temp_z = imu->accel_ms2[Z] - accel_bias_ms2[Z];

    imu->accel_ms2[X] = accel_calib_matrix[0][0] * temp_x + accel_calib_matrix[0][1] * temp_y + accel_calib_matrix[0][2] * temp_z;
    imu->accel_ms2[Y] = accel_calib_matrix[1][0] * temp_x + accel_calib_matrix[1][1] * temp_y + accel_calib_matrix[1][2] * temp_z;
    imu->accel_ms2[Z] = accel_calib_matrix[2][0] * temp_x + accel_calib_matrix[2][1] * temp_y + accel_calib_matrix[2][2] * temp_z;
}