/**
 * @file remote.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for remote.c file
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_H
#define __REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define CHANNEL_MAX 7

/* Exported Variables --------------------------------------------------------*/
extern uint32_t rvalue[CHANNEL_MAX];
extern uint8_t flymode;

/* Exported functions prototypes ---------------------------------------------*/
void REMOTE_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_H */
