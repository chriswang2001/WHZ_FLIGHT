/**
 * @file pos.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for pos.c file
 * @version 1.0
 * @date 2022-01-06
 * @copyright Copyright (c) 2022
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POS_H
#define __POS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported Variables --------------------------------------------------------*/
extern FloatVector3 pos, vel, accelEarth; // position and velocity estimator, accel cm/s2 in earth coordinate system

/* Exported functions prototypes ---------------------------------------------*/
void POS_Update();

#ifdef __cplusplus
}
#endif

#endif /* __POS_H */