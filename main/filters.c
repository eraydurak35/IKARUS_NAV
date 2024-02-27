#include <stdio.h>
#include "filters.h"
#include "math.h"
#include "noise_analize.h"


static const float fir_gain[7] = 
{
  0.142386908288827163f,
  0.142856919905681196f,
  0.143139372760745009f,
  0.143233598089493264f,
  0.143139372760745009f,
  0.142856919905681196f,
  0.142386908288827163f
};
/* static const float fir_gain[5] = 
{
    0.199736703181241426f,
    0.200131617212090052f,
    0.200263359213336989f,
    0.200131617212090052f,
    0.199736703181241426f
}; */


void fir_filter_init(fir_filter_t *fir)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        //fir[i].size = 5;
        fir[i].size = 7;
        fir[i].index = 0;
    }
}


void fir_filter(fir_filter_t *fir, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size) fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
    if (sumIndex > 0) sumIndex--;
    else sumIndex = fir->size - 1;
    temp += fir_gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}



void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size) fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
    if (sumIndex > 0) sumIndex--;
    else sumIndex = fir->size - 1;
    temp += gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}


void apply_fir_filter_to_imu(lsm6dsl_t *imu, fir_filter_t *fir)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        fir_filter(&fir[i], &imu->gyro_dps[i]);
        fir_filter(&fir[i+3], &imu->accel_ms2[i]);
    }
}




void notch_filter_init(notch_filter_t *notch)
{
    for (uint8_t i = 0; i < 18; i++)
    {
        notch[i].sample_rate = 1000.0f;
        notch_configure(200.0f, 25.0f, &notch[i]);
    }
    
}


//Q = notchCenterFreq_hz / bandwidth_hz
void notch_configure(float cf, float bw, notch_filter_t *notch)
{
    float Q = cf / bw;
    float omega = 2.0f * M_PI * cf / notch->sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);

    notch->b0 = 1.0f;
    notch->b1 = -2.0f * cs;
    notch->b2 = 1.0f;
    notch->a0 = 1.0f + alpha;
    notch->a1 = -2.0f * cs;
    notch->a2 = 1.0f - alpha;

    // prescale flter constants
    notch->b0 /= notch->a0;
    notch->b1 /= notch->a0;
    notch->b2 /= notch->a0;
    notch->a1 /= notch->a0;
    notch->a2 /= notch->a0;
}

// perform one filtering step
void notch_filter(notch_filter_t *notch, float *value)
{
    static float x = 0;
    static float y = 0;
    x = *value;
    y = notch->b0 * x + notch->b1 * notch->x1 + notch->b2 * notch->x2 - notch->a1 * notch->y1 - notch->a2 * notch->y2;
    notch->x2 = notch->x1;
    notch->x1 = x;
    notch->y2 = notch->y1;
    notch->y1 = y;
    *value = y;
}


void reconfig_all_notch_filters(notch_filter_t *notch, fft_t *fft)
{
    uint8_t x = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        for (uint8_t k = 0; k < 3; k++)
        {
            notch_configure(fft[i].filt_bin[k] * FFT_RES, 25.0f, &notch[x]);
            x++;
        }
    }
}


void apply_notch_filter_to_imu(lsm6dsl_t *imu, notch_filter_t *notch)
{
    for (uint8_t i = 0; i < 3; ++i) 
    {
        for (uint8_t j = 0; j < 3; ++j) 
        {
            notch_filter(&notch[i * 3 + j], &imu->gyro_dps[i]);
            notch_filter(&notch[9 + i * 3 + j], &imu->accel_ms2[i]);
        }
    }

}


void lowpass_configure(float cf, lowpass_filter_t *lowpass)
{
    float omega = 2.0f * M_PI * cf / 1000.0f;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / 2.0f;

    lowpass->b0 = (1.0f - cs) / 2.0f;
    lowpass->b1 = 1.0f - cs;
    lowpass->b2 = (1.0f - cs) / 2.0f;
    lowpass->a0 = 1.0f + alpha;
    lowpass->a1 = -2.0f * cs;
    lowpass->a2 = 1.0f - alpha;

    lowpass->b0 /= lowpass->a0;
    lowpass->b1 /= lowpass->a0;
    lowpass->b2 /= lowpass->a0;
    lowpass->a1 /= lowpass->a0;
    lowpass->a2 /= lowpass->a0;
}

void lowpass_filter(lowpass_filter_t *lowpass, float *value)
{
    static float x = 0;
    static float y = 0;
    x = *value;
    y = lowpass->b0 * x + lowpass->b1 * lowpass->x1 + lowpass->b2 * lowpass->x2 - lowpass->a1 * lowpass->y1 - lowpass->a2 * lowpass->y2;
    lowpass->x2 = lowpass->x1;
    lowpass->x1 = x;
    lowpass->y2 = lowpass->y1;
    lowpass->y1 = y;
    *value = y;
}