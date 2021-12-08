/**
 * @file pid.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Implementation of cascade PID
 * @version 1.0
 * @date 2021-11-18
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "ahrs.h"
#include "arm_math.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "sensor.h"
#include "stm32f4xx.h"

/* Variables -----------------------------------------------------------------*/
const float MAX_PID_OUTPUT = 400.f;
control_t rate_out, angle_out, set;

// arm_pid_instance_f32 PID_angle_roll = {
//     .Kp = 2.0f,
//     .Ki = 0.0f,
//     .Kd = 0.1f,
// };

// arm_pid_instance_f32 PID_rate_roll = {
//	   .Kp = 2.3f,
//     .Ki = 0.008f,
//     .Kd = 2.0f,
// };

// arm_pid_instance_f32 PID_angle_pitch = {
//     .Kp = 2.0f,
//     .Ki = 0.0f,
//     .Kd = 0.1f,
// };

// arm_pid_instance_f32 PID_rate_pitch = {
//	   .Kp = 2.3f,
//     .Ki = 0.008f,
//     .Kd = 2.0f,
// };

// arm_pid_instance_f32 PID_rate_yaw = {
//     .Kp = 7.0f,
//     .Ki = 0.005f,
//     .Kd = 0.0f,
// };

// arm_pid_instance_f32 PID_altitude = {
//     .Kp = 0.0f,
//     .Ki = 0.0f,
//     .Kd = 0.0f,
// };

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
    .Kp = 1.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
};

arm_pid_instance_f32 PID_rate_pitch = {
    .Kp = 3.2f,
    .Ki = 0.01f,
    .Kd = 0.5f,
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

/**
 * @brief Init pid parameter
 */
void PID_Init(void)
{
    arm_pid_init_f32(&PID_angle_roll, 1);
    arm_pid_init_f32(&PID_rate_roll, 1);

    arm_pid_init_f32(&PID_angle_pitch, 1);
    arm_pid_init_f32(&PID_rate_pitch, 1);

    arm_pid_init_f32(&PID_rate_yaw, 1);

    arm_pid_init_f32(&PID_altitude, 1);
}

float PID_Calculate(arm_pid_instance_f32 *S, float32_t in)
{
    float out = arm_pid_f32(S, in);

    if (out > MAX_PID_OUTPUT)
    {
        out = MAX_PID_OUTPUT;
        S->state[2] = out;
    }
    else if (out < -MAX_PID_OUTPUT)
    {
        out = -MAX_PID_OUTPUT;
        S->state[2] = out;
    }

    return out;
}

static inline float PWM_TO_ANGLE(uint32_t pwm)
{
    return ((float)pwm - 1500) * MAX_ANGLE / 500.f;
}

/**
 * @brief update pid control, angle at 125Hz, rate at 250Hz
 */
void PID_Upadte(void)
{
    if (mode == LOCKED)
        return;

    static uint8_t count = 0;

    set.roll = PWM_TO_ANGLE(rvalue[ROL]);
    set.pitch = PWM_TO_ANGLE(rvalue[PIT]);
    set.yaw = PWM_TO_ANGLE(rvalue[YAW]);
    set.altitude = rvalue[THR];

    if (count % 2 == 0)
    {
        angle_out.roll = PID_Calculate(&PID_angle_roll, set.roll - attitude.angle.roll);
        angle_out.pitch = PID_Calculate(&PID_angle_pitch, set.pitch + attitude.angle.pitch);
    }

    rate_out.roll = PID_Calculate(&PID_rate_roll, angle_out.roll - gyro.axis.y);
    rate_out.pitch = PID_Calculate(&PID_rate_pitch, angle_out.pitch + gyro.axis.x);
    //	rate_out.roll = PID_Calculate(&PID_rate_roll, set.roll - gyro.axis.y);
    //	rate_out.pitch = PID_Calculate(&PID_rate_pitch, set.pitch + gyro.axis.x);
    rate_out.yaw = PID_Calculate(&PID_rate_yaw, set.yaw + gyro.axis.z);
    rate_out.altitude = set.altitude;

    count++;

    MOTOR_Control(&rate_out);
}
