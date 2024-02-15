#include <stdio.h>
#include "lsm6dsl.h"
#include "i2c.h"


const float accel_bias_ms2[3] = {-0.023902f, -0.051104f, 0.238851f};
const float accel_calib_matrix[3][3] = {
  {0.994289f, 0.000270f, -0.002591f},
  {0.000270f, 0.995928f, 0.001289f},
  {-0.002591f, 0.001289f, 1.002714f}
};
static void getCalibratedResults(lsm6dsl_t *imu);
static void parse_buffer(uint8_t *buff, lsm6dsl_t *imu);


void lsm6dsl_setup(uint8_t odr, uint8_t accel_range, uint8_t gyro_range)
{
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL1_XL, odr << 4 | accel_range << 2 | 3); // 0xAF
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL8_XL, 0xE1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL2_G, odr << 4 | gyro_range << 2 | 0);// 0X98
    vTaskDelay(20 / portTICK_PERIOD_MS); 
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL4_C, 0x02);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, LSM6DSL_ADDR, CTRL6_C, 0x42);
    vTaskDelay(20 / portTICK_PERIOD_MS);
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

    temp_x = imu->accel_ms2[X] - accel_bias_ms2[X];
    temp_y = imu->accel_ms2[Y] - accel_bias_ms2[Y];
    temp_z = imu->accel_ms2[Z] - accel_bias_ms2[Z];

    imu->accel_ms2[X] = accel_calib_matrix[0][0] * temp_x + accel_calib_matrix[0][1] * temp_y + accel_calib_matrix[0][2] * temp_z;
    imu->accel_ms2[Y] = accel_calib_matrix[1][0] * temp_x + accel_calib_matrix[1][1] * temp_y + accel_calib_matrix[1][2] * temp_z;
    imu->accel_ms2[Z] = accel_calib_matrix[2][0] * temp_x + accel_calib_matrix[2][1] * temp_y + accel_calib_matrix[2][2] * temp_z;
}