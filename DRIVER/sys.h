/**
 * @file sys.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for sys.c file
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS_H
#define __SYS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void SYS_Init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_H */
