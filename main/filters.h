#ifndef FILTERS_H
#define FILTERS_H

#include <stdio.h>
#include "typedefs.h"

void fir_filter_init(fir_filter_t *fir);
void fir_filter(fir_filter_t *fir, float *value);
void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value);
void apply_fir_filter_to_imu(lsm6dsl_t *imu, fir_filter_t *fir);

void notch_filter_init(notch_filter_t *notch);
void notch_configure(float cf, float bw, notch_filter_t *notch);
void notch_filter(notch_filter_t *notch, float *value);
void apply_notch_filter_to_imu(lsm6dsl_t *imu, notch_filter_t *notch);
void reconfig_all_notch_filters(notch_filter_t *notch, fft_t *fft);


void lowpass_configure(float cf, lowpass_filter_t *lowpass);
void lowpass_filter(lowpass_filter_t *lowpass, float *value);
#endif