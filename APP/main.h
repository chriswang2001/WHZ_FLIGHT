/**
 * @file main.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for main.c file
 *        This file contains the common defines of the application
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "sys.h"

/* Exported types ------------------------------------------------------------*/
enum
{
    ROL = 0,
    PIT,
    THR,
    YAW,
    LOCK,
    LAND,
    MODE,
};

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
