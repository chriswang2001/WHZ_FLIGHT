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
#include "adc.h"
#include "ahrs.h"
#include "ano.h"
#include "motor.h"
#include "remote.h"
#include "sensor.h"
#include "ucos_ii.h"
#include "usart.h"
#include <stdio.h>

// START task
// Set task priority
#define START_TASK_PRIO 10 // Start task has the lowest priority
// Set task stack size
#define START_STK_SIZE 128
// Task task stack
OS_STK START_TASK_STK[START_STK_SIZE];
// Task function
void START_Task(void *pdata);

/**
 * @brief The application entry point.
 * @return int
 */
int main(void)
{
    SYS_Init();

    OSInit();                                                                                            // UCOS initialization
    OSTaskCreate(START_Task, (void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO); // Create start task
    OSStart();                                                                                           // Start task
}

/**
 * @brief Start task: sys init and create other tasks
 * @param pdata
 */
void START_Task(void *pdata)
{
    OSStatInit(); // Open statistics task

#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif

    OS_ENTER_CRITICAL(); // Enter critical area (close interrupt)

    OS_EXIT_CRITICAL(); // Exit critical area (open interrupt)

    OSTaskDel(START_TASK_PRIO); // Delete start task
}

void ANO_Task(void *pdata)
{
    static uint8_t data[300];
    static uint8_t cnt = 0;
    const uint8_t sensor = 4, euler = 1, battery = 10, pwm = 1, control = 1, remote = 5;

    while (1)
    {
        uint8_t size = 0;
        cnt++;
        if (cnt % sensor == 0)
            size += ANO_Send_Sensor(data + size, accel.axis.x * 1000, accel.axis.y * 1000, accel.axis.z * 1000, gyro.axis.x * 1000, gyro.axis.y * 1000, gyro.axis.z * 1000);
        if (cnt % euler == 0)
            size += ANO_Send_Euler(data + size, attitude.angle.roll, attitude.angle.pitch, attitude.angle.yaw);
        if (cnt % battery == 0)
        {
            if (HAL_ADC_GetState(&hadc1) == HAL_OK)
                size += ANO_Send_Battery(data + size, HAL_ADC_GetValue(&hadc1), OSCPUUsage);
            HAL_ADC_Start(&hadc1);
        }
        if (cnt % pwm == 0)
            size += ANO_Send_PWM(data + size, mvalue[0], mvalue[1], mvalue[2], mvalue[3]);
        // if (cnt % control == 0)
        //     size += ANO_Send_Control(data + size, float roll, float pitch, uint16_t thrust, int16_t yaw);
        if (cnt % remote == 0)
            size += ANO_Send_Remote(data + size, (int16_t*)rvalue, CHANNEL_MAX);

        HAL_UART_Transmit_DMA(&huart2, data, size);

        OSTimeDly(1);
    }
}
