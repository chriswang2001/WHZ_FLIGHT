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
#include "pos.h"
#include "remote.h"
#include "sensor.h"
#include "stm32f4xx.h"

/* Variables -----------------------------------------------------------------*/
const float MAX_PID_OUTPUT = 400.f;
control_t rate_out, angle_out, set;

arm_pid_instance_f32 PID_angle_roll = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.1f,
};

arm_pid_instance_f32 PID_rate_roll = {
    .Kp = 2.3f,
    .Ki = 0.008f,
    .Kd = 2.0f,
};

arm_pid_instance_f32 PID_angle_pitch = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.1f,
};

arm_pid_instance_f32 PID_rate_pitch = {
    .Kp = 2.3f,
    .Ki = 0.008f,
    .Kd = 2.0f,
};

arm_pid_instance_f32 PID_rate_yaw = {
    .Kp = 7.0f,
    .Ki = 0.005f,
    .Kd = 0.0f,
};

// arm_pid_instance_f32 PID_angle_roll = {
//     .Kp = 0.0f,
//     .Ki = 0.0f,
//     .Kd = 0.0f,
// };

// arm_pid_instance_f32 PID_rate_roll = {
//     .Kp = 0.0f,
//     .Ki = 0.0f,
//     .Kd = 0.0f,
// };

// arm_pid_instance_f32 PID_angle_pitch = {
//     .Kp = 0.0f,
//     .Ki = 0.0f,
//     .Kd = 0.0f,
// };

// arm_pid_instance_f32 PID_rate_pitch = {
//     .Kp = 5.f,
//     .Ki = 0.f,
//     .Kd = 0.f,
// };

// arm_pid_instance_f32 PID_rate_yaw = {
//     .Kp = 0.0f,
//     .Ki = 0.0f,
//     .Kd = 0.0f,
// };

arm_pid_instance_f32 PID_speed_altitude = {
    .Kp = 3.f,
    .Ki = 0.001f,
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

    arm_pid_init_f32(&PID_speed_altitude, 1);
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

static inline float PWM_TO_RATE(uint32_t pwm)
{
    return ((float)pwm - 1500) * MAX_RATE / 500.f;
}

static inline float PWM_TO_SPEED(uint32_t pwm)
{
    if (rvalue[THR] > 1700)
        return ((float)pwm - 1700) * MAX_SPEED / 300.f;
    else if (rvalue[THR] < 1300)
        return ((float)pwm - 1300) * MAX_SPEED / 300.f;
    else
        return 0.f;
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
    set.yaw = PWM_TO_RATE(rvalue[YAW]);

    if (count % 2 == 0)
    {
        angle_out.roll = PID_Calculate(&PID_angle_roll, set.roll - attitude.angle.roll);
        angle_out.pitch = PID_Calculate(&PID_angle_pitch, set.pitch + attitude.angle.pitch);
    }

    rate_out.roll = PID_Calculate(&PID_rate_roll, angle_out.roll - gyro.axis.y);
    rate_out.pitch = PID_Calculate(&PID_rate_pitch, angle_out.pitch + gyro.axis.x);
    // rate_out.roll = PID_Calculate(&PID_rate_roll, set.roll - gyro.axis.y);
    // rate_out.pitch = PID_Calculate(&PID_rate_pitch, set.pitch + gyro.axis.x);
    rate_out.yaw = PID_Calculate(&PID_rate_yaw, set.yaw + gyro.axis.z);
    if (mode == ALTITUDE)
    {
        set.altitude = PWM_TO_SPEED(rvalue[THR]);
        PID_speed_altitude.state[2] = rate_out.altitude;
        rate_out.altitude = PID_Calculate(&PID_speed_altitude, set.altitude - vel.axis.z);
    }
    else
        rate_out.altitude = rvalue[THR];

    if (rate_out.altitude > 1800)
        rate_out.altitude = 1800;

    count++;

    MOTOR_Control(&rate_out);
}
