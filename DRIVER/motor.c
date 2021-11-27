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
#include "tim.h"

/* Defines -------------------------------------------------------------------*/
#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_LIMIT(_PWM_) \
    _PWM_ < PWM_MIN ? PWM_MIN : (_PWM_ > PWM_MAX ? PWM_MAX : _PWM_)

uint16_t mvalue[4];

/**
 * @brief Motor Initialize: config high level value to 1ms
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
 * @brief Set motor rotaton rate by PID control
 * @param control control value caulated by PID controller
 */
void MOTOR_Control(control_t *control)
{
    float a = control->altitude, p = control->pitch, r = control->roll, y = control->yaw;
    mvalue[0] = a - p + r + y;
    mvalue[1] = a - p - r - y;
    mvalue[2] = a + p - r + y;
    mvalue[3] = a + p + r - y;
    MOTOR_Set(mvalue[0], mvalue[1], mvalue[2], mvalue[3]);
}
