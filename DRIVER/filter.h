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
float biquadFilterApply(biquadFilter_t *filter, float input);

#ifdef __cplusplus
}
#endif

#endif /* __FLITER_H */
