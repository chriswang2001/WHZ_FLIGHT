/**
 * @file filer.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-05
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <arm_math.h>

/* Defines -------------------------------------------------------------------*/
#define BIQUAD_Q 1.0f / sqrtf(2.0f) /* quality factor - butterworth*/

/* Exported types ------------------------------------------------------------*/
typedef struct biquadFilter_s /* this holds the data required to update samples thru a filter */
{
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;

typedef enum
{
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

/* Exported Variables --------------------------------------------------------*/
extern biquadFilter_t accelFilterLPF[3];
extern biquadFilter_t gyroFilterLPF[3];

/* Exported functions prototypes ---------------------------------------------*/
void FILTER_Init();
float flimitApply(float in, float min, float max);
int32_t deadBandApply(int32_t input, int32_t deadband);
float fdeadBandApply(float input, float deadband);
float slidingFilterApply(float *buffer, float in, int size);
float biquadFilterApply(biquadFilter_t *filter, float input);
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType);

#ifdef __cplusplus
}
#endif

#endif /* __FLITER_H */
