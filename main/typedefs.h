#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <stdio.h>


#define X 0
#define Y 1
#define Z 2

#define BUFF_SIZE 130

typedef struct
{
  int16_t range_cm;
} range_finder_t;

typedef struct
{
  float notch_1_freq;
  float notch_1_bandwidth;
  float notch_2_freq;
  float notch_2_bandwidth;
  float ahrs_filter_beta;
  float ahrs_filter_zeta;
  float alt_filter_beta;
  float velz_filter_beta;
  float velz_filter_zeta;
  float velxy_filter_beta;
  float mag_declination_deg;
} nav_config_t;

typedef struct
{
  uint8_t arm_status;
  uint8_t alt_hold_status;
  uint8_t pos_hold_status;
  uint8_t takeoff_status;
  uint8_t waypoint_mission_status;
} flight_t;

typedef struct
{
  int16_t pitch;
  int16_t roll;
  uint16_t heading;
  int16_t pitch_dps;
  int16_t roll_dps;
  int16_t yaw_dps;
} data_1_t;
typedef struct
{
  int16_t velocity_x_ms;
  int16_t velocity_y_ms;
  int16_t velocity_z_ms;
  int16_t flow_x_velocity_ms;
  int16_t flow_y_velocity_ms;
  uint8_t flow_quality;
  int16_t altitude;
  int16_t baro_altitude;
  int16_t barometer_pressure;
  uint16_t barometer_temperature;
} data_2_t;

typedef struct
{
  uint16_t imu_temperature;
  int16_t acc_x_ned_ms2;
  int16_t acc_y_ned_ms2;
  int16_t acc_z_ned_ms2;
  int16_t acc_x_ms2;
  int16_t acc_y_ms2;
  int16_t acc_z_ms2;
  int16_t mag_x_gauss;
  int16_t mag_y_gauss;
  int16_t mag_z_gauss;
} data_3_t;

////////////////////////////////////////////////////



typedef struct
{
  float par_t1;
  float par_t2;
  float par_t3;
  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;
} bmp390_calib_t;

typedef struct
{
    int32_t pressADC;
    int32_t tempADC;
    float temp;
    float press;
    float gnd_press;
    float altitude_m;
    float velocity_ms;
    float init_temp;
} bmp390_t;


typedef struct
{
    uint8_t size;
    float buffer[7];
    uint8_t index;
} fir_filter_t;

typedef struct
{
    float sample_rate;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float x1;
    float x2;
    float y1;
    float y2;
} notch_filter_t;



typedef struct
{
    float axis[3];
} hmc5883l_t;


typedef struct
{
    float gyro_dps[3];
    float accel_ms2[3];
    int16_t temp_mC;
    float gyro_bias_dps[3];
} lsm6dsl_t;



typedef struct
{
    float real;
    float imag;
} complex_t;

typedef struct
{
    float cosx;
    float sinx;
} fft_gain_t;

typedef struct
{
    uint32_t psd[32];
    int32_t buff[64];
    int32_t hann_buff[64];
    float filt_bin[3];
    struct PEAKS
    {
        float bin;
        uint32_t value;
    } peaks[3];
} fft_t;


typedef struct
{
  int16_t raw_x_cpi, raw_y_cpi;
  float filt_x_cpi, filt_y_cpi;
  float velocity_x_ms, velocity_y_ms;
  float quality;
} pmw3901_t;


typedef struct 
{
    float w, x, y, z;
} quat_t;

typedef struct
{
    float x, y, z;
} vector_t;


typedef struct 
{
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float pitch_dps;
    float roll_dps;
    float yaw_dps;
    float altitude_m;
    float vel_forward_ms;
    float vel_right_ms;
    float vel_up_ms;
    float acc_forward_ms2;
    float acc_right_ms2;
    float acc_up_ms2;
} states_t;



typedef struct
{
    uint8_t data[BUFF_SIZE];
    uint8_t lenght;
} uart_data_t;





#endif