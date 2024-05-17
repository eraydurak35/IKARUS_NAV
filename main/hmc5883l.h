#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdio.h>
#include "typedefs.h"

#define HMC5883L_ADDR 0x1E
#define CONFIG_REG_A 0x0
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define X_MSB_REG 0x03

#define AVERAGE_1 (0 << 5)
#define AVERAGE_2 (1 << 5)
#define AVERAGE_4 (2 << 5)
#define AVERAGE_8 (3 << 5)

#define ODR_0_75HZ (0 << 2)
#define ODR_1_5HZ (1 << 2)
#define ODR_3HZ (2 << 2)
#define ODR_7_5HZ (3 << 2)
#define ODR_15HZ (4 << 2)
#define ODR_30HZ (5 << 2)
#define ODR_75HZ (6 << 2)

#define MODE_NORMAL_HMC 0
#define MODE_POS_BIAS_HMC 1
#define MODE_NEG_BIAS_HMC 2

// 0 00 1 Average   000 0.75Hz DOR    00 Normal
// 0 01 2 Average   001 1.5Hz  DOR    01 Pozitive Bias Mode
// 0 10 4 Average   010 3Hz    DOR    10 Negative Bias Mode
// 0 11 8 Average   011 7.5Hz  DOR
//                  100 15Hz   DOR
//                  101 30Hz   DOR
//                  110 75Hz   DOR

#define RES_0_8_GAUSS (0 << 5)
#define RES_1_3_GAUSS (1 << 5)
#define RES_1_9_GAUSS (2 << 5)
#define RES_2_5_GAUSS (3 << 5)
#define RES_4_0_GAUSS (4 << 5)
#define RES_4_7_GAUSS (5 << 5)
#define RES_5_6_GAUSS (6 << 5)
#define RES_8_1_GAUSS (7 << 5)

// 000 0.88 Gauss   00000
// 001 1.3  Gauss   00000
// 010 1.9  Gauss   00000
// 011 2.5  Gauss   00000
// 100 4    Gauss   00000
// 101 4.7  Gauss   00000
// 110 5.6  Gauss   00000
// 111 8.1  Gauss   00000

#define HS_I2C_ENABLE (1 << 7)
#define HS_I2C_DISABLE (0 << 7)

#define MEAS_CONTINUOUS 0
#define MEAS_SINGLE 1

// 0 Disable HS(3400kHz) I2C  00000   00 Continuous Measurement
// 1 Enable HS(3400kHz) I2C   00000   01 Single Measurement




// HMC5883L, default address 0x1E

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  �0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  �1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  �1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  �2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  �4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  �4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  �5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  �8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */


void hmc5883l_setup(float *mg_cal);
void hmc5883l_read(magnetometer_t *hmc, magnetometer_t *uncalib_hmc);

#endif