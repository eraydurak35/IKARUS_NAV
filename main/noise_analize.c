#include <stdio.h>
#include <math.h>
#include "noise_analize.h"
#include "typedefs.h"




complex_t fft_out[6][FFT_SIZE];

const float hann_gains[32] = 
{
  0.080000f, 0.082286f, 0.089121f, 0.100437f, 0.116121f, 0.136018f, 0.159930f, 0.187620f, 
  0.218811f, 0.253195f, 0.290429f, 0.330143f, 0.371943f, 0.415413f, 0.460122f, 0.505624f, 
  0.551468f, 0.597198f, 0.642360f, 0.686504f, 0.729192f, 0.770000f, 0.808522f, 0.844375f, 
  0.877204f, 0.906681f, 0.932514f, 0.954446f, 0.972259f, 0.985776f, 0.994862f, 0.999428f
};


void noise_analize_init(fft_t *fft)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        for (uint8_t k = 0; k < 3; k++)
        {
            fft[i].filt_bin[k] = START_BIN;
        }
    }
}

void sample_imu_to_analize(lsm6dsl_t *imu, fft_t *fft)
{

    for (uint8_t k = 0; k < 3; k++)
    {

        for (uint8_t i = 0; i < FFT_SIZE - 1; i++) 
        {
            fft[k].buff[i] = fft[k].buff[i + 1];
            fft[k+3].buff[i] = fft[k+3].buff[i + 1];
        }
        fft[k].buff[FFT_SIZE - 1] = (int32_t)(imu->gyro_dps[k] * 1000.0f);
        fft[k+3].buff[FFT_SIZE - 1] = (int32_t)(imu->accel_ms2[k] * 1000000.0f);
    }
}


void analize_imu_noise(fft_t *fft)
{
    static uint8_t fft_state = APPLY_HANN_WINDOW;

    if (fft_state == APPLY_HANN_WINDOW)
    {
        for (uint8_t k = 0; k < 6; k++)
        {
            for (uint8_t i = 0; i < FFT_SIZE; i++)
            {
                if (i > (FFT_SIZE/2) - 1)
                {
                    fft[k].hann_buff[i] = fft[k].buff[i] * hann_gains[(FFT_SIZE - 1) - i];
                }
                else
                {
                    fft[k].hann_buff[i] = fft[k].buff[i] * hann_gains[i];
                }
            }
        }

        fft_state = CALCULATE_FFT_GYR;
    }
    else if (fft_state == CALCULATE_FFT_GYR)
    {
        compute_fft_3(fft[0].hann_buff, fft[1].hann_buff, fft[2].hann_buff, fft_out[0], fft_out[1], fft_out[2], FFT_SIZE);
        fft_state = CALCULATE_FFT_ACC;
    }
    else if (fft_state == CALCULATE_FFT_ACC)
    {
        compute_fft_3(fft[3].hann_buff, fft[4].hann_buff, fft[5].hann_buff, fft_out[3], fft_out[4], fft_out[5], FFT_SIZE);
        fft_state = FIND_PEAK;
    }
    else if (fft_state == FIND_PEAK)
    {

        for (uint8_t k = 0; k < 6; k++)
        {
            for (uint8_t i = START_BIN; i < END_BIN; i++)
            {
                fft[k].psd[i] = (uint32_t)(fabs(fft_out[k][i].real) + fabs(fft_out[k][i].imag));
            }
            find_peaks(&fft[k]);
        }

        fft_state = APPLY_HANN_WINDOW;
    }
}

void compute_fft_3(int32_t* input_1, int32_t* input_2, int32_t* input_3, complex_t* output_1, complex_t* output_2, complex_t* output_3, uint8_t N) 
{
    if (N <= 1) 
    {
        output_1[0].real = (float)input_1[0];
        output_1[0].imag = 0.0f;

        output_2[0].real = (float)input_2[0];
        output_2[0].imag = 0.0f;

        output_3[0].real = (float)input_3[0];
        output_3[0].imag = 0.0f;
        return;
    }

    int32_t even_1[N / 2];
    int32_t odd_1[N / 2];

    int32_t even_2[N / 2];
    int32_t odd_2[N / 2];

    int32_t even_3[N / 2];
    int32_t odd_3[N / 2];

    complex_t evenResult_1[N / 2];
    complex_t oddResult_1[N / 2];

    complex_t evenResult_2[N / 2];
    complex_t oddResult_2[N / 2];

    complex_t evenResult_3[N / 2];
    complex_t oddResult_3[N / 2];


    for (uint8_t i = 0; i < N / 2; ++i) 
    {
        even_1[i] = input_1[i * 2];
        odd_1[i] = input_1[i * 2 + 1];

        even_2[i] = input_2[i * 2];
        odd_2[i] = input_2[i * 2 + 1];

        even_3[i] = input_3[i * 2];
        odd_3[i] = input_3[i * 2 + 1];
    }

    compute_fft_3(even_1, even_2, even_3, evenResult_1, evenResult_2, evenResult_3, N / 2);
    compute_fft_3(odd_1, odd_2, odd_3, oddResult_1, oddResult_2, oddResult_3, N / 2);

    for (uint8_t k = 0; k < N / 2; ++k) 
    {
        fft_gain_t fft_gain;

        if (k == 0)
        {
            fft_gain.cosx = 1.0f;
            fft_gain.sinx = 0.0f;
        }
        else
        {
            get_fft_gain(&fft_gain, k, N);
        }

        
        float t_real_1 = fft_gain.cosx * oddResult_1[k].real - fft_gain.sinx * oddResult_1[k].imag;
        float t_imag_1 = fft_gain.sinx * oddResult_1[k].real + fft_gain.cosx * oddResult_1[k].imag;

        float t_real_2 = fft_gain.cosx * oddResult_2[k].real - fft_gain.sinx * oddResult_2[k].imag;
        float t_imag_2 = fft_gain.sinx * oddResult_2[k].real + fft_gain.cosx * oddResult_2[k].imag;

        float t_real_3 = fft_gain.cosx * oddResult_3[k].real - fft_gain.sinx * oddResult_3[k].imag;
        float t_imag_3 = fft_gain.sinx * oddResult_3[k].real + fft_gain.cosx * oddResult_3[k].imag;

        output_1[k].real = evenResult_1[k].real + t_real_1;
        output_1[k].imag = evenResult_1[k].imag + t_imag_1;

        output_2[k].real = evenResult_2[k].real + t_real_2;
        output_2[k].imag = evenResult_2[k].imag + t_imag_2;

        output_3[k].real = evenResult_3[k].real + t_real_3;
        output_3[k].imag = evenResult_3[k].imag + t_imag_3;

        output_1[k + N / 2].real = evenResult_1[k].real - t_real_1;
        output_1[k + N / 2].imag = evenResult_1[k].imag - t_imag_1;

        output_2[k + N / 2].real = evenResult_2[k].real - t_real_2;
        output_2[k + N / 2].imag = evenResult_2[k].imag - t_imag_2;

        output_3[k + N / 2].real = evenResult_3[k].real - t_real_3;
        output_3[k + N / 2].imag = evenResult_3[k].imag - t_imag_3;
    }
}


void find_peaks(fft_t *fft)
{
    // Get memory ready for new peak data on current axis
    for (uint8_t i = 0; i < PEAK_COUNT; i++)
    {
        fft->peaks[i].bin = START_BIN;
        fft->peaks[i].value = 0;
    }

    // Search for N biggest peaks in frequency spectrum
    for (uint8_t bin = START_BIN; bin < END_BIN; bin++)
    {
        // Check if bin is peak
        if ((fft->psd[bin] > fft->psd[bin - 1]) && (fft->psd[bin] > fft->psd[bin + 1]))
        {
            // Check if peak is big enough to be one of N biggest peaks.
            // If so, insert peak and sort peaks in descending height order
            for (uint8_t p = 0; p < PEAK_COUNT; p++)
            {
                if (fft->psd[bin] > fft->peaks[p].value)
                {
                    for (int k = PEAK_COUNT - 1; k > p; k--)
                    {
                        fft->peaks[k] = fft->peaks[k-1];
                    }
                    fft->peaks[p].bin = bin;
                    fft->peaks[p].value = fft->psd[bin];
                    break;
                }
            }
            bin++; // If bin is peak, next bin can't be peak => skip it
        }
    }

    // Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
    for (uint8_t p = PEAK_COUNT - 1; p > 0; p--) 
    {
        for (uint8_t k = 0; k < p; k++) 
        {
            // Swap peaks but ignore swapping void peaks (bin = 0). This leaves
            // void peaks at the end of peaks array without moving them
            if (fft->peaks[k].bin > fft->peaks[k + 1].bin && fft->peaks[k + 1].bin != 0) 
            {
                float temp_bin = fft->peaks[k].bin;
                uint32_t temp_value = fft->peaks[k].value;

                fft->peaks[k].bin = fft->peaks[k + 1].bin;
                fft->peaks[k].value = fft->peaks[k + 1].value;

                fft->peaks[k + 1].bin = temp_bin;
                fft->peaks[k + 1].value = temp_value;
            }
        }
    }


    for (uint8_t i = 0; i < PEAK_COUNT; i++)
    {
        // Height of peak bin (y1) and shoulder bins (y0, y2)
        float y0 = fft->psd[(uint8_t)(fft->peaks[i].bin) - 1];
        float y1 = fft->psd[(uint8_t)(fft->peaks[i].bin)];
        float y2 = fft->psd[(uint8_t)(fft->peaks[i].bin) + 1];

        // Estimate true peak position aka. meanBin (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
        float denom = 2.0f * (y0 - (2.0f * y1) + y2);
        if (denom != 0)
        {
            fft->peaks[i].bin += (y0 - y2) / denom;
        }
    }

    for (uint8_t i = 0; i < PEAK_COUNT; i++)
    {
        fft->filt_bin[i] += (fft->peaks[i].bin - fft->filt_bin[i]) * 0.05f;
    }

}

void get_fft_gain(fft_gain_t *gain, uint8_t k, uint8_t n)
{
    uint16_t q = 10000*k/n;
        
    switch(q)
    {
        case 2500:                    //31 times
            gain->cosx = 0.0f;
            gain->sinx = -1.0f;
            break;
        case 1250:                    //15 times
            gain->cosx = 0.707106781f;
            gain->sinx = -0.707106781f;
            break;
        case 3750:                    //15 times
            gain->cosx = -0.707106781f;
            gain->sinx = -0.707106781f;
            break;
        case 625:                     //7 times
            gain->cosx = 0.923879532f;
            gain->sinx = -0.382683432f;
            break;
        case 1875:                    //7 times
            gain->cosx = 0.382683432f;
            gain->sinx = -0.923879532f;
            break;
        case 3125:                    //7 times
            gain->cosx = -0.382683432f;
            gain->sinx = -0.923879532f;
            break;
        case 4375:                    //7 times
            gain->cosx = -0.923879532f; 
            gain->sinx = -0.382683432f;
            break;
        case 312:                      //3 times
            gain->cosx = 0.98078528f;
            gain->sinx = -0.195090322f;
            break;
        case 937:                     //3 times
            gain->cosx = 0.831469612f;
            gain->sinx = -0.555570233f;
            break;
        case 1562:                    //3 times
            gain->cosx = 0.555570233f;
            gain->sinx = -0.831469612f;
            break;
        case 2187:                     //3 times
            gain->cosx = 0.195090322f;
            gain->sinx = -0.98078528f;
            break;
        case 2812:                     //3 times
            gain->cosx = -0.195090322f;
            gain->sinx = -0.98078528f;
            break;
        case 3437:                     //3 times
            gain->cosx = -0.555570233f;
            gain->sinx = -0.831469612f;
            break;
        case 4062:                     //3 times
            gain->cosx = -0.831469612f;
            gain->sinx = -0.555570233f;
            break;
        case 4687:                     //3 times
            gain->cosx = -0.98078528f;
            gain->sinx = -0.195090322f;
            break;
        default:                       //16 times
            float a = MINUS_2PI * (float)k / (float)n;
            gain->cosx = cosf(a);
            gain->sinx = sinf(a);
    }
}