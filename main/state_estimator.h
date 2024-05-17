#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdio.h>
#include "typedefs.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f
#define RANGE_BARO_TRANS_START_ALT 4.0f // 4
#define RANGE_BARO_TRANS_END_ALT 6.0f // 6
#define GYRO_MOVEMENT_DETECT_THRESHOLD 15.0f
#define IN_FLT_MAG_ALLN_ALT 2.0f
#define ACC_UP_BIAS_PREDICT_LIM 0.3f


void estimator_init(nav_config_t *cfg, states_t *sta, lsm6dsl_t *lsm, magnetometer_t *hmc, bmp390_t *baro, pmw3901_t *flw, range_finder_t *rng, flight_t *flt);
void ahrs_predict();
void ahrs_correct();
void get_earth_frame_accel();
void correct_velocityXY();
void predict_velocityXY();
void get_flow_velocity();
void estimate_altitude_velocity();

#endif /*STATE_ESTIMATOR_H*/