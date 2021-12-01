/**
 * @file motor.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for motor.c file
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Exported functions prototypes ---------------------------------------------*/
void MOTOR_Init(void);
void MOTOR_Set(uint16_t M1_PWM, uint16_t M2_PWM, uint16_t M3_PWM, uint16_t M4_PWM);
void MOTOR_Control(control_t *control);
bool MOTOR_LockCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
