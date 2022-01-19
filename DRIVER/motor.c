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
#define PWM_LIMIT(_PWM_) _PWM_ < PWM_MIN ? PWM_MIN : (_PWM_ > PWM_MAX ? PWM_MAX : _PWM_)

#define PWM_BASE1 1000
#define PWM_BASE2 1000
#define PWM_BASE3 1000
#define PWM_BASE4 1000

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
 * @brief Set motor rotaton rate by PID control
 * @param control control value caulated by PID controller
 */
void MOTOR_Control(control_t *control)
{
    static uint16_t M1, M2, M3, M4;
    M1 = PWM_BASE1 + control->altitude - control->pitch + control->roll + control->yaw;
    M2 = PWM_BASE2 + control->altitude - control->pitch - control->roll - control->yaw;
    M3 = PWM_BASE3 + control->altitude + control->pitch - control->roll + control->yaw;
    M4 = PWM_BASE4 + control->altitude + control->pitch + control->roll - control->yaw;

    MOTOR_Set(M1, M2, M3, M4);
}

/**
 * @brief Lock motor by setting pwm to 1000
 */
void MOTOR_Locked(void)
{
    MOTOR_Set(1000, 1000, 1000, 1000);
}
