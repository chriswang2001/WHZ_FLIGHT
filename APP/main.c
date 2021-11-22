/**
 * @file main.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Main program body and task function
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sensor.h"
#include <stdio.h>

/**
 * @brief The application entry point.
 * @return int
 */
int main(void)
{
    SYS_Init();

    printf("test\n");

    while (1)
    {
        Sensor_Task();
        HAL_Delay(20);
    }
}
