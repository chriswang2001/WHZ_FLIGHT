/**
 * @file main.h
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Header for main.c file containing the common defines and types of the application
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
#include "ucos_ii.h"

/* Exported Variables --------------------------------------------------------*/
extern uint32_t deltaTick;
extern float freqFlight;

/* Defines -------------------------------------------------------------------*/
#define ANO_SEND // define to send information to ano host
//#define DEBUG_ENABLE // define to enable debug printf

// START Task
#define START_TASK_PRIO 10    // Set task priority, Start task has the lowest priority
#define START_STK_SIZE 128    // Set task stack size
void START_Task(void *pdata); // Task function

// ANO Task
#define ANO_TASK_PRIO 9     // Set task priority
#define ANO_STK_SIZE 128    // Set task stack size
#define ANO_CYCLE_MS 10     // Set task cycle
void ANO_Task(void *pdata); // Task function

// FLIGHT Task
#define FLIGHT_TASK_PRIO 8     // Set task priority
#define FLIGHT_STK_SIZE 256    // Set task stack size
#define FLIGHT_CYCLE_MS 2      // Set task cycle
void FLIGHT_Task(void *pdata); // Task function

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Remote channel name
 */
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

/**
 * @brief Flymode name
 */
enum
{
    LOCKED = 0,
    STABILIZE,
    ALTITUDE,
    LOITER,
    LANDING,
};

/**
 * @brief Three-dimensional spacial vector in form of int16_t
 */
typedef union
{
    int16_t array[3];

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } axis;
} Int16Vector3;

/**
 * @brief Three-dimensional spacial vector in form of float
 */
typedef union
{
    float array[3];

    struct
    {
        float x;
        float y;
        float z;
    } axis;
} FloatVector3;

/**
 * @brief Euler angles union in the Aerospace sequence, also known as the ZYX sequence.
 */
typedef union
{
    float array[3];

    struct
    {
        float roll;
        float pitch;
        float yaw;
    } angle;
} EulerAngles;

/**
 * @brief Control value struct
 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float altitude;
} control_t;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
