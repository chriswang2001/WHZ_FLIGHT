/**
 * @file main.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Main program body
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ahrs.h"
#include "ano.h"
#include "motor.h"
#include "pid.h"
#include "remote.h"
#include "sensor.h"
#include "tim.h"
#include "ucos_ii.h"
#include "usart.h"

/* Defines -------------------------------------------------------------------*/
// START Task
#define START_TASK_PRIO 10                                         // Set task priority, Start task has the lowest priority
#define START_STK_SIZE 128                                         // Set task stack size
__attribute__((aligned(8))) OS_STK START_TASK_STK[START_STK_SIZE]; // Task task stack
void START_Task(void *pdata);                                      // Task function

// ANO Task
#define ANO_TASK_PRIO 9                                        // Set task priority
#define ANO_STK_SIZE 128                                       // Set task stack size
__attribute__((aligned(8))) OS_STK ANO_TASK_STK[ANO_STK_SIZE]; // Task task stack
void ANO_Task(void *pdata);                                    // Task function

// FLIGHT Task
#define FLIGHT_TASK_PRIO 8                                           // Set task priority
#define FLIGHT_STK_SIZE 256                                          // Set task stack size
__attribute__((aligned(8))) OS_STK FLIGHT_TASK_STK[FLIGHT_STK_SIZE]; // Task task stack
void FLIGHT_Task(void *pdata);                                       // Task function

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
 * @brief Start Task: sys init and create other tasks
 * @param pdata
 */
void START_Task(void *pdata)
{
    OSStatInit(); // Open statistics task

#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif

    OS_ENTER_CRITICAL();                                                                                     // Enter critical area (close interrupt)
    OSTaskCreateExt(ANO_Task, (void *)0, (OS_STK *)&ANO_TASK_STK[ANO_STK_SIZE - 1], ANO_TASK_PRIO, 1, &ANO_TASK_STK[0], ANO_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);                   // Create ANO Task
    OSTaskCreateExt(FLIGHT_Task, (void *)0, (OS_STK *)&FLIGHT_TASK_STK[FLIGHT_STK_SIZE - 1], FLIGHT_TASK_PRIO, 2, &FLIGHT_TASK_STK[0], FLIGHT_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR); // Create FLIGHT Task
    OS_EXIT_CRITICAL();                                                                                      // Exit critical area (open interrupt)

    OSTaskDel(START_TASK_PRIO); // Delete start task
}

/**
 * @brief ANO Taks: send necessary information to ano host
 * @param pdata
 */
void ANO_Task(void *pdata)
{
#define MAX_DATA_SIZE 300

    static uint8_t count = 0;
    static uint8_t data[MAX_DATA_SIZE];
    static const uint8_t sensor = 5, sensor2 = 7, euler = 1, battery = 50, pwm = 1, control = 1, remote = 6;

    while (1)
    {
        count++;
        uint16_t size = 0;

        if (count % sensor == 0)
            size += ANO_Send_Sensor(data + size, accel.axis.x * 1000, accel.axis.y * 1000, accel.axis.z * 1000, gyro.axis.x * 100, gyro.axis.y * 100, gyro.axis.z * 100);
        if (count % sensor2 == 0)
            size += ANO_Send_Sensor2(data + size, mag.axis.x * 10, mag.axis.y * 10, mag.axis.z * 10, altitude * 100.f, 0);
        if (count % euler == 0)
            size += ANO_Send_Euler(data + size, attitude.angle.roll, attitude.angle.pitch, attitude.angle.yaw);
        if (count % battery == 0)
            size += ANO_Send_Battery(data + size, voltage, OSCPUUsage);
        if (count % pwm == 0)
            size += ANO_Send_PWM(data + size, __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4));
        if (count % control == 0)
            size += ANO_Send_Control(data + size, rate_out.roll, rate_out.pitch, rate_out.altitude, rate_out.yaw);
        if (count % remote == 0)
            size += ANO_Send_Remote(data + size, rvalue[0], rvalue[1], rvalue[2], rvalue[3], rvalue[4], rvalue[5], rvalue[6], 0, 0, 0);

        if (size <= MAX_DATA_SIZE)
            HAL_UART_Transmit_DMA(&huart2, data, size);
        else
        {
            char string[] = "send data overflow";
            ANO_Send_String(string, sizeof(string), GREEN);
        }

        OSTimeDly(10);
    }
}

/**
 * @brief Flight Task: estimation of orientation and pid control
 * @param pdata
 */
void FLIGHT_Task(void *pdata)
{
    while (1)
    {
        SENSOR_Update();
        AHRS_Update();
        PID_Upadte();

        OSTimeDly(5);
    }
}
