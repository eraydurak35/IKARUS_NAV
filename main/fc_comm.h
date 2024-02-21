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






/* #include "uart.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define data1_header1 100
#define data1_header2 101
#define data1_footer 102

#define data2_header1 90
#define data2_header2 91
#define data2_footer 92

#define data3_header1 80
#define data3_header2 81
#define data3_footer 82

#define ins_config_header1 70
#define ins_config_header2 71
#define ins_config_footer 72

#define flight_header1 60
#define flight_header2 61
#define flight_footer 62

#define range_finder_header1 50
#define range_finder_header2 51
#define range_finder_footer 52



void fc_comm_init(states_t *std, flight_t *flt, range_finder_t *rng, nav_config_t *cfg, lsm6dsl_t *lsm, hmc5883l_t *hmc, pmw3901_t *pmw, bmp390_t *bro);
void parse_fc_data(uart_data_t *uart_buff);
void send_data1();
void send_data2();
void send_data3();
void check_flight_status(); */

#endif