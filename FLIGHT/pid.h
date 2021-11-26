/**
 * @file pid.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for pid.c file
 * @version 1.0
 * @date 2021-11-18
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MAX_ANGLE 35

void PID_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
