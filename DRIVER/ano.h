/**
 * @file ano.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for ano.c file
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANO_H
#define __ANO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Defines -------------------------------------------------------------------*/
#define black 0
#define red 1
#define green 2

/* Exported functions prototypes ---------------------------------------------*/
uint8_t ANO_Send_Sensor(uint8_t *data, int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z);
uint8_t ANO_Send_Euler(uint8_t *data, float angle_rol, float angle_pit, float angle_yaw);
uint8_t ANO_Send_Battery(uint8_t *data, float voltage, int16_t usage);
uint8_t ANO_Send_PWM(uint8_t *data, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4);
uint8_t ANO_Send_Control(uint8_t *data, float roll, float pitch, uint16_t thrust, int16_t yaw);
uint8_t ANO_Send_Remote(uint8_t *data, int16_t *ch, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* __ANO_H */
