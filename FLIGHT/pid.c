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
#include "filter.h"
#include "main.h"
#include "motor.h"
#include "pos.h"
#include "remote.h"
#include "sensor.h"
#include "stm32f4xx.h"
#include <stdlib.h>

/* Variables -----------------------------------------------------------------*/
control_t rate_out, angle_out, set;

PID_type PID_angle_roll = {
    .kp = 2.4f,
    .ki = 0.0f,
    .kd = 0.0f,
};

PID_type PID_angle_pitch = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
};

PID_type PID_rate_roll = {
    .kp = 2.0f,
    .ki = 0.22f,
    .kd = 0.005f,
};

PID_type PID_rate_pitch = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
};

PID_type PID_rate_yaw = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
};

PID_type PID_speed_altitude = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
};

// PID_type PID_angle_roll = {
//     .kp = 2.4,
//     .ki = 0.0,
//     .kd = 0.0,
// };

// PID_type PID_angle_pitch = {
//     .kp = 2.4,
//     .ki = 0.0,
//     .kd = 0.0,
// };

// PID_type PID_rate_roll = {
//     .kp = 2.0,
//     .ki = 0.22,
//     .kd = 0.005,
// };

// PID_type PID_rate_pitch = {
//     .kp = 2.0,
//     .ki = 0.22,
//     .kd = 0.005,
// };

// PID_type PID_rate_yaw = {
//     .kp = 4.0,
//     .ki = 0.8,
//     .kd = 0.0,
// };

// PID_type PID_speed_altitude = {
//     .kp = 3.f,
//     .ki = 0.001f,
//     .kd = 0.0f,
// };

/* Defines -------------------------------------------------------------------*/
#define ANGLE_COUNT 2

#define MAX_ANGLE 30.f  // deg
#define MAX_RATE 50.f   // deg/s
#define MAX_SPEED 100.f // cm/s2

/*中点死区值*/
#define DEAD_BAND 5

/*角速度PID积分限幅（单位：deg/s）*/
#define PID_RATE_INTEGRATION_LIMIT 200.0f
#define PID_RATE_YAW_INTEGRATION_LIMIT 100.0f

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_OUTPUT_LIMIT 500.0f
#define PID_RATE_YAW_OUTPUT_LIMIT 300.0f

/*角度PID积分限幅（单位：deg）*/
#define PID_ANGLE_INTEGRATION_LIMIT 0.0f
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 0.0f

/*角度PID输出限幅（单位：deg/s）*/
#define PID_ANGLE_OUTPUT_LIMIT 300.0f
#define PID_ANGLE_YAW_OUTPUT_LIMIT 150.0f

// Z轴速度PID积分限幅（单位cm/s）
#define PID_VZ_INTEGRATION_LIMIT 500.0f
// Z轴速度PID输出限幅（单位油门值）
#define PID_VZ_OUTPUT_LIMIT 800.0f

//角速度PID D项低通截止频率（单位Hz）
#define PID_RATE_LPF_CUTOFF_FREQ 80.0

// Z轴速度PID D项低通截止频率（单位Hz）
#define PID_VZ_LPF_CUTOFF_FREQ 15.0

void PID_Type_Init(PID_type *pid, float iLimit, float oLimit, bool enableDFilter, float filterFreq, float cutoffFreq)
{
    pid->prevError = 0;
    pid->integ = 0;
    pid->iLimit = iLimit;
    pid->oLimit = oLimit;
    pid->enableDFilter = enableDFilter;
    if (pid->enableDFilter)
    {
        biquadFilterInit(&pid->dFilter, filterFreq, cutoffFreq, BIQUAD_Q, FILTER_LPF);
    }
}

/**
 * @brief Init pid parameter
 */
void PID_Init(void)
{
    PID_Type_Init(&PID_angle_roll, PID_ANGLE_INTEGRATION_LIMIT, PID_ANGLE_OUTPUT_LIMIT, false, 0, 0);
    PID_Type_Init(&PID_rate_roll, PID_RATE_INTEGRATION_LIMIT, PID_RATE_OUTPUT_LIMIT, true, freqFlight, PID_RATE_LPF_CUTOFF_FREQ);

    PID_Type_Init(&PID_angle_pitch, PID_ANGLE_INTEGRATION_LIMIT, PID_ANGLE_OUTPUT_LIMIT, false, 0, 0);
    PID_Type_Init(&PID_rate_pitch, PID_RATE_INTEGRATION_LIMIT, PID_RATE_OUTPUT_LIMIT, true, freqFlight, PID_RATE_LPF_CUTOFF_FREQ);

    PID_Type_Init(&PID_rate_yaw, PID_RATE_YAW_INTEGRATION_LIMIT, PID_RATE_YAW_OUTPUT_LIMIT, true, freqFlight, PID_RATE_LPF_CUTOFF_FREQ);

    PID_Type_Init(&PID_speed_altitude, PID_VZ_INTEGRATION_LIMIT, PID_VZ_OUTPUT_LIMIT, true, freqFlight, PID_VZ_LPF_CUTOFF_FREQ);
}

void PID_Type_Reset(PID_type *pid)
{
    pid->prevError = 0;
    pid->integ = 0;
}

void PID_Reset(void)
{
    PID_Type_Reset(&PID_angle_roll);
    PID_Type_Reset(&PID_rate_roll);

    PID_Type_Reset(&PID_angle_pitch);
    PID_Type_Reset(&PID_rate_pitch);

    PID_Type_Reset(&PID_rate_yaw);

    PID_Type_Reset(&PID_speed_altitude);
}

float PID_Calculate(PID_type *pid, float32_t target, float state, float dt)
{
    int iflag = 1;
    float output = 0.0f;
    if (dt == 0)
        dt = FLIGHT_CYCLE_MS;
    pid->error = target - state;
    pid->integ += pid->error * dt;

    //积分限幅
    if (pid->iLimit != 0)
    {
        pid->integ = flimitApply(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->deriv = (pid->error - pid->prevError) / dt;
    if (pid->enableDFilter)
    {
        pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);
    }

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;

    output = pid->outP + pid->outI + pid->outD;

    //输出限幅
    if (pid->oLimit != 0)
    {
        output = flimitApply(output, -pid->oLimit, pid->oLimit);
    }

    pid->prevError = pid->error;

    return output;
}

static inline float PWM_TO_ANGLE(uint32_t pwm)
{
    return deadBandApply(pwm - 1500, DEAD_BAND) * MAX_ANGLE / 500.f;
}

static inline float PWM_TO_RATE(uint32_t pwm)
{
    return deadBandApply(pwm - 1500, DEAD_BAND) * MAX_RATE / 500.f;
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
    if (flymode == LOCKED)
    {
        PID_Reset();
        return;
    }

    static uint8_t count = 0;

    if (count % ANGLE_COUNT == 0)
    {
        set.roll = PWM_TO_ANGLE(rvalue[ROL]);
        set.pitch = PWM_TO_ANGLE(rvalue[PIT]);
        angle_out.roll = PID_Calculate(&PID_angle_roll, set.roll, attitude.angle.roll, deltaTick);
        angle_out.pitch = PID_Calculate(&PID_angle_pitch, set.pitch, -attitude.angle.pitch, deltaTick);
    }

    set.yaw = PWM_TO_RATE(rvalue[YAW]);
    rate_out.roll = PID_Calculate(&PID_rate_roll, angle_out.roll, gyro.axis.y, deltaTick);
    rate_out.pitch = PID_Calculate(&PID_rate_pitch, angle_out.pitch, -gyro.axis.x, deltaTick);
    rate_out.yaw = PID_Calculate(&PID_rate_yaw, set.yaw, -gyro.axis.z, deltaTick);

    if (flymode == ALTITUDE)
    {
        set.altitude = PWM_TO_SPEED(rvalue[THR]);
        rate_out.altitude = PID_Calculate(&PID_speed_altitude, set.altitude, vel.axis.z, deltaTick);
    }
    else
    {
        rate_out.altitude = rvalue[THR] - 1000.f;
        if (rate_out.altitude > PID_VZ_OUTPUT_LIMIT)
            rate_out.altitude = PID_VZ_OUTPUT_LIMIT;
    }

    MOTOR_Control(&rate_out);
    count++;
}
