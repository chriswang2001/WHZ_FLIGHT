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
#include "i2c.h"
#include "pid.h"
#include "pos.h"
#include "remote.h"
#include "sensor.h"
#include "tim.h"
#include "ucos_ii.h"
#include "usart.h"

/* Defines -------------------------------------------------------------------*/
#define ANO_MAX_SIZE 300 // maximum size to send

/* Variables -----------------------------------------------------------------*/
uint32_t deltaTick;                          // tick between two flight task in ms
float freqFlight = 1000.f / FLIGHT_CYCLE_MS; // flight task frequency in Hz

// task stack alligned 8 to avoid printf float data error
__attribute__((aligned(8))) OS_STK START_TASK_STK[START_STK_SIZE];
__attribute__((aligned(8))) OS_STK ANO_TASK_STK[ANO_STK_SIZE];
__attribute__((aligned(8))) OS_STK FLIGHT_TASK_STK[FLIGHT_STK_SIZE];

/**
 * @brief The application entry point.
 * @return int
 */
int main(void)
{
    SYS_Init();

    OSInit();
    OSTaskCreate(START_Task, (void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO);
    OSStart();
}

/**
 * @brief Start Task: Create tasks
 * @param pdata
 */
void START_Task(void *pdata)
{
    OSStatInit(); // Open statistics task

#if OS_CRITICAL_METHOD == 3u
    OS_CPU_SR cpu_sr;
#endif

    OS_ENTER_CRITICAL();
    // using task creat extend to enable TASK_OPT_STK_CHK
    OSTaskCreateExt(ANO_Task, (void *)0, (OS_STK *)&ANO_TASK_STK[ANO_STK_SIZE - 1], ANO_TASK_PRIO, 1, &ANO_TASK_STK[0], ANO_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);                   // Create ANO Task
    OSTaskCreateExt(FLIGHT_Task, (void *)0, (OS_STK *)&FLIGHT_TASK_STK[FLIGHT_STK_SIZE - 1], FLIGHT_TASK_PRIO, 2, &FLIGHT_TASK_STK[0], FLIGHT_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR); // Create FLIGHT Task
    OS_EXIT_CRITICAL();

    OSTaskDel(START_TASK_PRIO);
}

/**
 * @brief ANO Task: send necessary information to ano host
 * @param pdata
 */
void ANO_Task(void *pdata)
{
    static uint8_t count = 0;

    while (1)
    {
        count++;
        if (count == 0)
        {
            if (voltage < 10.5f && voltage > 8.5f)
            {
                char string[] = "low voltage, please return to base";
                ANO_Send_String(string, sizeof(string), RED);
            }
        }

#ifdef ANO_SEND
        uint16_t size = 0;
        static uint8_t data[ANO_MAX_SIZE];
        static const uint8_t sensor = 5, sensor2 = 7, euler = 1, mode = 8, battery = 50, pwm = 1, control = 2, remote = 6;

        if (count % sensor == 0)
            size += ANO_Send_Sensor(data + size, accel.axis.x * 1000, accel.axis.y * 1000, accel.axis.z * 1000, gyro.axis.x * 100, gyro.axis.y * 100, gyro.axis.z * 100);
        if (count % sensor2 == 0)
            size += ANO_Send_Sensor2(data + size, accelEarth.axis.z, vel.axis.z, pos.axis.z, altitude, temperature);
        if (count % euler == 0)
        {
            size += ANO_Send_Euler(data + size, attitude.angle.roll, attitude.angle.pitch, attitude.angle.yaw);
            size += ANO_Send_Target(data + size, set.roll, set.pitch, set.yaw);
        }
        if (count % mode == 0)
            size += ANO_Send_Mode(data + size, flymode, flymode > 1 ? 1 : flymode);
        if (count % battery == 0)
            size += ANO_Send_Battery(data + size, voltage, OSCPUUsage);
        if (count % pwm == 0)
            size += ANO_Send_PWM(data + size, __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4));
        if (count % control == 0)
            size += ANO_Send_Control(data + size, rate_out.roll, rate_out.pitch, rate_out.altitude, rate_out.yaw);
        if (count % remote == 0)
            size += ANO_Send_Remote(data + size, rvalue[ROL], rvalue[PIT], rvalue[THR], rvalue[YAW], rvalue[LOCK], rvalue[LAND], rvalue[MODE], 0, 0, 0);

        if (size <= ANO_MAX_SIZE)
            HAL_UART_Transmit_DMA(&huart2, data, size);
        else
        {
            char string[] = "Data Overflow";
            ANO_Send_String(string, sizeof(string), GREEN);
        }
#endif
        OSTimeDly(ANO_CYCLE_MS);
    }
}

/**
 * @brief Flight Task: estimation of orientation position and pid control
 * @param pdata
 */
void FLIGHT_Task(void *pdata)
{
    static uint32_t tickstart;
    tickstart = HAL_GetTick();

    while (1)
    {
        SENSOR_Update();
        deltaTick = HAL_GetTick() - tickstart;
        AHRS_Update();
        POS_Update();
        PID_Upadte();

        tickstart = HAL_GetTick();
        OSTimeDly(FLIGHT_CYCLE_MS);
    }
}
