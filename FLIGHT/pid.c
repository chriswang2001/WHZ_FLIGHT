/**
 * @file pid.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Implementation of cascade PID
 * @version 1.0
 * @date 2021-11-18
 * @copyright Copyright (c) 2021
 */

#include "pid.h"
#include "ahrs.h"
#include "arm_math.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "sensor.h"
#include "stm32f4xx.h"

const float PWMToAngle = MAX_ANGLE / 500.f;
const float PWMToRate = MAX_Rate / 500.f;

arm_pid_instance_f32 PID_angle_roll = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_rate_roll = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_angle_pitch = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_rate_pitch = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_rate_yaw = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_altitude = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

void PID_Init(void)
{
    arm_pid_init_f32(&PID_angle_roll, 1);
    arm_pid_init_f32(&PID_rate_roll, 1);

    arm_pid_init_f32(&PID_angle_pitch, 1);
    arm_pid_init_f32(&PID_rate_pitch, 1);

    arm_pid_init_f32(&PID_rate_yaw, 1);

    arm_pid_init_f32(&PID_altitude, 1);
}

void PID_Upadte(void)
{
    static uint8_t cnt = 0;
    if (rvalue[LOCK] < 1500)
    {
        MOTOR_Set(1000, 1000, 1000, 1000);
        return;
    }

    static control_t set, angle_out, rate_out;

    set.roll = (rvalue[ROL] - 1500) * PWMToAngle;
    set.pitch = (rvalue[PIT] - 1500) * PWMToAngle;
    set.yaw = (rvalue[YAW] - 1500) * PWMToRate;
    set.altitude = rvalue[THR];

    if (cnt % 2 == 0)
    {
        angle_out.roll = arm_pid_f32(&PID_angle_roll, set.roll - attitude.angle.roll);
        angle_out.pitch = arm_pid_f32(&PID_angle_pitch, set.pitch + attitude.angle.pitch);
    }

    rate_out.roll = arm_pid_f32(&PID_rate_roll, angle_out.roll - gyro.axis.y);
    rate_out.pitch = arm_pid_f32(&PID_rate_pitch, angle_out.pitch + gyro.axis.x);
    rate_out.yaw = arm_pid_f32(&PID_rate_yaw, set.yaw + gyro.axis.z);
    rate_out.altitude = set.altitude;

    cnt++;

    MOTOR_Control(&rate_out);
}
