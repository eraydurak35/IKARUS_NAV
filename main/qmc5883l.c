#include <stdio.h>
#include "qmc5883l.h"
#include "i2c.h"

static float *mag_calib_data = NULL;

static void parse_qmc5883l_data(magnetometer_t *qmc, uint8_t *buffer);
static void get_calibrated_result(magnetometer_t *qmc);
static void mag_throttle_correction(magnetometer_t *mag, uint16_t throttle);
static uint8_t qmc5883l_check_drdy_bit();

void qmc5883l_setup(float *mg_cal)
{
    mag_calib_data = mg_cal;

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_1, QMC5883L_ADDR, SET_RESET_REG, 0x01);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_1, QMC5883L_ADDR, 0x0A, 0x41);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_1, QMC5883L_ADDR, CTRL_REG_1, MODE_CONTINUOUS | ODR_50_HZ | OSR_512 | SCALE_8_GAUSS);
}

void qmc5883l_read(magnetometer_t *qmc, magnetometer_t *uncalib_qmc, uint16_t throttle)
{
    static uint8_t buff[6] = {0};

    if (qmc5883l_check_drdy_bit() == 1)
    {
        i2c_read_data(I2C_NUM_1, QMC5883L_ADDR, OUTPUT_X_REG, buff, 6);
        parse_qmc5883l_data(qmc, buff);
        mag_throttle_correction(qmc, throttle);
        *uncalib_qmc = *qmc;
        get_calibrated_result(qmc);
    }

}

static void mag_throttle_correction(magnetometer_t *mag, uint16_t throttle)
{
    static const float correction_vector[3] = {-0.0782f, 0.512f, -0.2185f};

    mag->axis[X] += correction_vector[X] * (throttle - 200);
    mag->axis[Y] += correction_vector[Y] * (throttle - 200);
    mag->axis[Z] += correction_vector[Z] * (throttle - 200);
}

static uint8_t qmc5883l_check_drdy_bit()
{
    static uint8_t buff[1] = {0};
    i2c_read_data(I2C_NUM_1, QMC5883L_ADDR, STATUS_REG, buff, 1);

    if (buff[0] == 1 || buff[0] == 4 || buff[0] == 5) return 1;
    return 0;

}

static void parse_qmc5883l_data(magnetometer_t *qmc, uint8_t *buffer)
{   
    // X Y Z order
    qmc->axis[X] = (float)((int16_t)(buffer[0] | buffer[1] << 8));
    qmc->axis[Y] = (float)((int16_t)(buffer[2] | buffer[3] << 8));
    qmc->axis[Z] = (float)((int16_t)(buffer[4] | buffer[5] << 8));
}

static void get_calibrated_result(magnetometer_t *qmc)
{
    static float temp_x = 0;
    static float temp_y = 0;
    static float temp_z = 0; 

    temp_x = qmc->axis[X] - mag_calib_data[X];
    temp_y = qmc->axis[Y] - mag_calib_data[Y];
    temp_z = qmc->axis[Z] - mag_calib_data[Z];

    qmc->axis[X] = mag_calib_data[3] * temp_x + mag_calib_data[4] * temp_y + mag_calib_data[5] * temp_z;
    qmc->axis[Y] = mag_calib_data[6] * temp_x + mag_calib_data[7] * temp_y + mag_calib_data[8] * temp_z;
    qmc->axis[Z] = mag_calib_data[9] * temp_x + mag_calib_data[10] * temp_y + mag_calib_data[11] * temp_z;
}

