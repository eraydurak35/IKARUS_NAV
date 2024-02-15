#ifndef NOISE_ANALIZE_H
#define NOISE_ANALIZE_H

#include <stdio.h>
#include "typedefs.h"

#define MINUS_2PI -2.0 * M_PI
#define APPLY_HANN_WINDOW 0
#define CALCULATE_FFT_GYR 1
#define CALCULATE_FFT_ACC 2
#define FIND_PEAK 3
#define FFT_RES 500.0f / FFT_SIZE
#define FFT_SIZE 64
#define START_BIN 7
#define END_BIN (FFT_SIZE / 2) - 1
#define PEAK_COUNT 3


void noise_analize_init(fft_t *fft);
void get_fft_gain(fft_gain_t *gain, uint8_t k, uint8_t n);
void sample_imu_to_analize(lsm6dsl_t *imu, fft_t *fft);
void analize_imu_noise(fft_t *fft);
void compute_fft_3(int32_t *input_1, int32_t *input_2, int32_t *input_3, complex_t* output_1, complex_t* output_2, complex_t* output_3, uint8_t N);
void find_peaks(fft_t *fft);
#endif