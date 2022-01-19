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
#include "filter.h"
#include "main.h"
#include <arm_math.h>
#include <stdbool.h>

typedef struct //定义记录每个方向pid各量的结构
{
    float error;            //< error
    float prevError;        //< previous error
    float integ;            //< integral
    float deriv;            //< derivative
    float kp;               //< proportional gain
    float ki;               //< integral gain
    float kd;               //< derivative gain
    float outP;             //< proportional output (debugging)
    float outI;             //< integral output (debugging)
    float outD;             //< derivative output (debugging)
    float iLimit;           //< integral limit
    float oLimit;           //< total PID output limit, absolute value. '0' means no limit.
    bool enableDFilter;     //< filter for D term enable flag
    biquadFilter_t dFilter; //< filter for D term
} PID_type;

/* Exported Variables --------------------------------------------------------*/
extern PID_type PID_angle_roll;
extern PID_type PID_rate_roll;
extern PID_type PID_angle_pitch;
extern PID_type PID_rate_pitch;
extern PID_type PID_rate_yaw;
extern PID_type PID_speed_altitude;
extern control_t rate_out, angle_out, set;

/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(void);
void PID_Upadte(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
