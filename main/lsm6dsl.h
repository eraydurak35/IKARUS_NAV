#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stdio.h>
#include "typedefs.h"

#define X 0
#define Y 1
#define Z 2

#define CTRL1_XL 0x10
#define CTRL8_XL 0x17
#define CTRL2_G 0x11
#define CTRL4_C 0x13
#define CTRL6_C 0x15
#define OUT_TEMP_L 0x20
#define LSM6DSL_ADDR 0x6A
#define LSM6DSL_WHO_AM_I 0x0F // expected_return 0x6A

#define GYRO_SENSITIVITY_DEGS 0.035f
#define ACCEL_SENSITIVITY_MS2 0.0023927f
#define TEMP_SENSITIVITY_MC 2.56f


#define ODR_12_5_HZ 1
#define ODR_26_HZ 2
#define ODR_52_HZ 3
#define ODR_104_HZ 4
#define ODR_208_HZ 5
#define ODR_416_HZ 6
#define ODR_833_HZ 7
#define ODR_1666_HZ 8
#define ODR_3300_HZ 9
#define ODR_6600_HZ 10

#define ACCEL_2G 0
#define ACCEL_4G 2
#define ACCEL_8G 3
#define ACCEL_16G 1

#define GYRO_250DPS 0
#define GYRO_500DPS 1
#define GYRO_1000DPS 2
#define GYRO_2000DPS 3


void lsmldsl_read(lsm6dsl_t *imu);
void lsm6dsl_setup(uint8_t odr, uint8_t accel_range, uint8_t gyro_range);


#endif