/**
 * @file ahrs.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for ahrs.c file
 * @version 1.0
 * @date 2021-11-17
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AHRS_H
#define __AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"

/* Exported variable declerations --------------------------------------------*/
extern EulerAngles attitude;
extern const float sampleFreq;
extern volatile float q0, q1, q2, q3;

/* Exported functions prototypes ---------------------------------------------*/
void AHRS_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __AHRS_H */
