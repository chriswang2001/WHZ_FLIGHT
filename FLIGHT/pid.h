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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <arm_math.h>

/* Defines -------------------------------------------------------------------*/
#define MAX_ANGLE 35

/* Exported Variables --------------------------------------------------------*/
extern arm_pid_instance_f32 PID_angle_roll;
extern arm_pid_instance_f32 PID_rate_roll;
extern arm_pid_instance_f32 PID_angle_pitch;
extern arm_pid_instance_f32 PID_rate_pitch;
extern arm_pid_instance_f32 PID_rate_yaw;
extern arm_pid_instance_f32 PID_altitude;
extern control_t rate_out;

/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(void);
void PID_Upadte(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
