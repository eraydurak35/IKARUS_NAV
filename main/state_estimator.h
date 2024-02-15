#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdio.h>
#include "typedefs.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f


void estimator_init(nav_config_t *cfg, states_t *sta, lsm6dsl_t *lsm, hmc5883l_t *hmc, bmp390_t *baro, pmw3901_t *flw, range_finder_t *rng);
void ahrs_predict();
void ahrs_correct();
void get_earth_frame_accel();
void correct_velocityXY();
void predict_velocityXY();
void get_flow_velocity();
void calculate_altitude_velocity();

#endif /*STATE_ESTIMATOR_H*/