/**
 * @file filer.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-05
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "filer.h"
#include "arm_math.h"

/* Defines -------------------------------------------------------------------*/
#define BIQUAD_Q 1.0f / sqrtf(2.0f) /* quality factor - butterworth*/

//二阶滤波器
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (samplingFreq / 2))
    {
        // setup variables
        const float sampleRate = samplingFreq;
        const float omega = 2.0f * PI * ((float)filterFreq) / sampleRate;
        const float sn = arm_sin_f32(omega);
        const float cs = arm_cos_f32(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType)
        {
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 = 1 - cs;
            b2 = (1 - cs) / 2;
            break;
        case FILTER_NOTCH:
            b0 = 1;
            b1 = -2 * cs;
            b2 = 1;
            break;
        }
        const float a0 = 1 + alpha;
        const float a1 = -2 * cs;
        const float a2 = 1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    }
    else
    {
        // Not possible to filter frequencies above Nyquist frequency - passthrough
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
    }

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

//二阶低通滤波器
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq)
{
    biquadFilterInit(filter, samplingFreq, filterFreq, BIQUAD_Q, FILTER_LPF);
}

// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}
