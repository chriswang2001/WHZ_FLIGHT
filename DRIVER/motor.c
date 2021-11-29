/**
 * @file motor.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Motor Controller by PWM
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "remote.h"
#include "tim.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_LIMIT(_PWM_) \
    _PWM_ < PWM_MIN ? PWM_MIN : (_PWM_ > PWM_MAX ? PWM_MAX : _PWM_)

/**
 * @brief Motor Initialize: set high level value to 1ms
 */
void MOTOR_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
 * @brief Set motor rotaton rate by PWM value
 * @param M1_PWM Motor1 PWM value, between 1000 and 2000
 * @param M2_PWM Motor2 PWM value, between 1000 and 2000
 * @param M3_PWM Motor3 PWM value, between 1000 and 2000
 * @param M4_PWM Motor4 PWM value, between 1000 and 2000
 */
void MOTOR_Set(uint16_t M1_PWM, uint16_t M2_PWM, uint16_t M3_PWM, uint16_t M4_PWM)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_LIMIT(M1_PWM));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_LIMIT(M2_PWM));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_LIMIT(M3_PWM));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_LIMIT(M4_PWM));
}

/**
 * @brief Lock motor when remote send lock value
 * @return true motor lock
 * @return false motor not lock
 */
bool MOTOR_LockCheck()
{
    if (rvalue[LOCK] < 1500)
    {
        MOTOR_Set(1000, 1000, 1000, 1000);
        return true;
    }

    return false;
}

/**
 * @brief Set motor rotaton rate by PID control
 * @param control control value caulated by PID controller
 */
void MOTOR_Control(control_t *control)
{
    if (MOTOR_LockCheck())
        return;

    float a = control->altitude, p = control->pitch, r = control->roll, y = control->yaw;
    MOTOR_Set(a - p + r + y, a - p - r - y, a + p - r + y, a + p + r - y);
}
