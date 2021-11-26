/**
 * @file pid.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Implementation of cascade PID
 * @version 1.0
 * @date 2021-11-18
 * @copyright Copyright (c) 2021
 */

#include "pid.h"
#include "arm_math.h"
#include "main.h"
#include "remote.h"
#include "stm32f4xx.h"

const float scale = MAX_ANGLE / 500.f;

extern float roll, roll_rate;

arm_pid_instance_f32 outer_roll = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 inner_roll = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

void PID_Init(void)
{
    arm_pid_init_f32(&outer_roll, 1);
    arm_pid_init_f32(&inner_roll, 1);
}

void PID_Taks()
{
    //     float roll_in = (rvalue[ROL] - 1500) * scale;

    //     float roll_out1 = arm_pid_f32(&outer_roll, roll_in - roll);

    //     float roll_out = arm_pid_f32(&inner_roll, roll_out1 - roll_rate);
}
