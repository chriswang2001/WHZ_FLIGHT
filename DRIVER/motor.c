/**
 * @file motor.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Motor Controller by PWM
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Defines -------------------------------------------------------------------*/
#define PWM_MAX 2000

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
    if (M1_PWM > PWM_MAX) M1_PWM = PWM_MAX;
    if (M2_PWM > PWM_MAX) M2_PWM = PWM_MAX;
    if (M3_PWM > PWM_MAX) M3_PWM = PWM_MAX;
    if (M4_PWM > PWM_MAX) M4_PWM = PWM_MAX;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, M1_PWM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, M2_PWM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, M3_PWM);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, M4_PWM);
}
