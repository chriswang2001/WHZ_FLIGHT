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
#include "ucos_ii.h"
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

    // SYS_Init();
    // printf("test\n");

    // while (1)
    // {
    // }
}

/**
 * @brief Start task: sys init and create other tasks
 * @param pdata
 */
void START_Task(void *pdata)
{
    OS_CPU_SR cpu_sr = 0;
    OSStatInit(); // Open statistics task

    OS_ENTER_CRITICAL(); // Enter critical area (close interrupt)

    OS_EXIT_CRITICAL(); // Exit critical area (open interrupt)

    OSTaskDel(START_TASK_PRIO); // Delete start task
}
