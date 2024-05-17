#ifndef FC_COMM_H
#define FC_COMM_H

#include <stdio.h>
#include "typedefs.h"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define RECV_HEADER_1 0x10
#define RECV_HEADER_2 0x11
#define RECV_HEADER_3 0x12
#define RECV_HEADER_4 0x13
#define FOOTER 0x31
#define SEND_HEADER 0x69

void flight_comm_init(states_t *sts, bmp390_t *bmp, pmw3901_t *pmw, lsm6dsl_t *lsm, magnetometer_t *mag, flight_t *flt, nav_config_t *cfg, range_finder_t *rng, float *mg_cal, float *acc_cal);
void slave_send_recv_flight_comm();

#endif