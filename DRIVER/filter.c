/**
 * @file filer.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief digital data filter
 * @version 1.0
 * @date 2021-12-05
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include "ahrs.h"
#include "arm_math.h"

/* Defines -------------------------------------------------------------------*/
#define ACCEL_LPF_CUTOFF_FREQ 15.0f
#define GYRO_LPF_CUTOFF_FREQ 80.0f

/* Variables -----------------------------------------------------------------*/
biquadFilter_t accelFilterLPF[3];
biquadFilter_t gyroFilterLPF[3];

/**
 * @brief limit value
 * @param in sample data to filter
 * @param min limit minimum
 * @param max limit maximum
 * @return float data after filtering
 */
float flimitApply(float in, float min, float max)
{
    return in < min ? min : (in > max ? max : in);
}

/**
 * @brief apply deadband int
 * @param input sample data to filter
 * @param deadband deadband value
 * @return int32_t float data after filtering
 */
int32_t deadBandApply(int32_t input, int32_t deadband)
{
    if (abs(input) < deadband)
    {
        input = 0;
    }
    else if (input > 0)
    {
        input -= deadband;
    }
    else if (input < 0)
    {
        input += deadband;
    }

    return input;
}

/**
 * @brief apply deadband to float
 * @param input sample data to filter
 * @param deadband deadband value
 * @return float data after filtering
 */
float fdeadBandApply(float input, float deadband)
{
    if ((float)fabs(input) < deadband)
    {
        input = 0;
    }
    else if (input > 0)
    {
        input -= deadband;
    }
    else if (input < 0)
    {
        input += deadband;
    }

    return input;
}

/**
 * @brief sliding window filter also kown as moving average filter
 * @param buffer slide window buffer
 * @param input sample data to filter
 * @param size sizeof slide window
 * @return float data after filtering
 */
float slidingFilterApply(float *buffer, float input, int size)
{
    // check if buffer is initialized
    if (buffer[0])
    {
        float filter_sum = 0;
        for (int i = 1; i < size - 1; i++)
        {
            buffer[i] = buffer[i + 1];
            filter_sum += buffer[i];
        }
        buffer[size - 1] = input;
        filter_sum += buffer[size - 1];

        return (filter_sum / ((float)size - 1));
    }
    else
    {
        buffer[0] = 1.f;
        for (int i = 1; i < size; i++)
        {
            buffer[i] = input;
        }

        return input;
    }
}

/**
 * @brief biquad IIR filter
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
 * @brief Filter Gain Initialize
 */
void FILTER_Init()
{
    for (int axis = 0; axis < 3; axis++)
    {
        biquadFilterInit(&gyroFilterLPF[axis], freqFlight, GYRO_LPF_CUTOFF_FREQ, BIQUAD_Q, FILTER_LPF);
        biquadFilterInit(&accelFilterLPF[axis], freqFlight, ACCEL_LPF_CUTOFF_FREQ, BIQUAD_Q, FILTER_LPF);
    }
}
