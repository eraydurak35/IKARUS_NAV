#ifndef FC_COMM_H
#define FC_COMM_H

#include <stdio.h>
#include "typedefs.h"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define HEADER 0x69
#define FOOTER 0x31

void flight_comm_init(states_t *sts, bmp390_t *bmp, pmw3901_t *pmw, lsm6dsl_t *lsm, hmc5883l_t *hmc, flight_t *flt, nav_config_t *cfg, range_finder_t *rng);
void slave_send_recv_flight_comm();

#endif