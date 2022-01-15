/**
 * @file filer.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-05
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include "ahrs.h"
#include "arm_math.h"

/* Defines -------------------------------------------------------------------*/
#define BIQUAD_Q 1.0f / sqrtf(2.0f) /* quality factor - butterworth*/
#define ACCEL_LPF_CUTOFF_FREQ 15.0f
#define GYRO_LPF_CUTOFF_FREQ 80.0f

/* Variables -----------------------------------------------------------------*/
biquadFilter_t accelFilterLPF[3];
biquadFilter_t gyroFilterLPF[3];

/* Function prototypes -------------------------------------------------------*/
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType);

/**
 * @brief Filter Gain Initialize
 */
void FILTER_Init()
{
    for (int axis = 0; axis < 3; axis++)
    {
        biquadFilterInit(&gyroFilterLPF[axis], sampleFreq, GYRO_LPF_CUTOFF_FREQ, BIQUAD_Q, FILTER_LPF);
        biquadFilterInit(&accelFilterLPF[axis], sampleFreq, ACCEL_LPF_CUTOFF_FREQ, BIQUAD_Q, FILTER_LPF);
    }
}

/**
 * @brief Initialize Biquad Filter
 * @param filter filter instance
 * @param samplingFreq sample frequency
 * @param filterFreq cut off frequency
 * @param Q quality factor
 * @param filterType lowpass or highpass
 */
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

/**
 * @brief filter sample data
 * @param filter filter instance
 * @param input sample data to filter
 * @return float data after filtering
 */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;

    return result;
}
