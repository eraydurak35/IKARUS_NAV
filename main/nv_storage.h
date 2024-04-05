#ifndef STORAGE_H
#define STORAGE_H

#include "typedefs.h"

void read_config(nav_config_t *cfg);
void save_config(nav_config_t *cfg);
void print_config(nav_config_t cfg);
void save_mag_cal(float *mg_cal);
void read_mag_cal(float *mg_cal);
void save_acc_cal(float *acc_cal);
void read_acc_cal(float *acc_cal);
#endif