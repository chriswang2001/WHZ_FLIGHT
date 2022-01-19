/**
 * @file remote.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Remote Controller by input capture
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "remote.h"
#include "motor.h"
#include "tim.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define ARR_VALUE1 0xffff
#define ARR_VALUE2 0xffffffff

/* Variables -----------------------------------------------------------------*/
uint8_t flymode;                                                        // flight mode;
bool rstate[CHANNEL_MAX];                                               // caputure state 0:now is low level 1:now is high level
uint32_t rvalue[CHANNEL_MAX], pvalue[CHANNEL_MAX], ucount[CHANNEL_MAX]; // caputre value, pre caputre value, update count

/**
 * @brief Remote Controller Initialize
 */
void REMOTE_Init(void)
{
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
}

const uint32_t TIM_CHANNEL[CHANNEL_MAX] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
};

const uint32_t TIM_FLAG[CHANNEL_MAX] = {
    TIM_FLAG_CC1,
    TIM_FLAG_CC2,
    TIM_FLAG_CC3,
    TIM_FLAG_CC4,
    TIM_FLAG_CC1,
    TIM_FLAG_CC2,
    TIM_FLAG_CC3,
};

/**
 * @brief handle timer capture compare
 * @param cc cpture channel
 * @param htim the pointer cpture timer
 */
static inline void TIM_CC_Handler(int8_t cc, TIM_HandleTypeDef *htim)
{
    if (cc < 0)
        return;

    if (rstate[cc]) // cpauture falling edge
    {
        uint32_t arr = (htim == &htim1) ? ARR_VALUE1 : ARR_VALUE2;
        uint32_t pwm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL[cc]) + arr * ucount[cc] - pvalue[cc];
        rvalue[cc] = pwm < 900 ? rvalue[cc] : (pwm > 2100 ? rvalue[cc] : pwm);
        rstate[cc] = 0;
        TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL[cc]);
        TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL[cc], TIM_ICPOLARITY_RISING);
    }
    else // cpauture rising edge
    {
        pvalue[cc] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL[cc]);
        ucount[cc] = 0;
        rstate[cc] = 1;
        TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL[cc]);
        TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL[cc], TIM_ICPOLARITY_FALLING);
    }
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG[cc]);
}

/**
 * @brief This function handles TIM1 capture compare interrupt.
 */
void TIM1_CC_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif
    OS_ENTER_CRITICAL();
    OSIntEnter(); /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    int8_t cc = -1;

    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) != RESET)
        cc = ROL;
    else if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2) != RESET)
        cc = PIT;
    else if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC3) != RESET)
        cc = THR;
    else if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) != RESET)
        cc = YAW;
    else
        HAL_TIM_IRQHandler(&htim1);

    TIM_CC_Handler(cc, &htim1);

    OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR            */
}

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif
    OS_ENTER_CRITICAL();
    OSIntEnter(); /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
    {
        for (int i = 0; i < 4; i++)
            ucount[i]++;
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    }
    else
        HAL_TIM_IRQHandler(&htim1);

    OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR            */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif
    OS_ENTER_CRITICAL();
    OSIntEnter(); /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    int8_t cc = -1;

    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        for (int i = 4; i < 7; i++)
            ucount[i]++;
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

        OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR            */
        return;
    }

    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET)
        cc = LOCK;
    else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET)
        cc = LAND;
    else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3) != RESET)
        cc = MODE;
    else
        HAL_TIM_IRQHandler(&htim2);

    TIM_CC_Handler(cc, &htim2);

    flymode = rvalue[MODE] < 1300 ? STABILIZE : (rvalue[MODE] < 1700 ? ALTITUDE : LOITER);
    if (rvalue[LAND] > 1500)
        flymode = LANDING;
    if (rvalue[LOCK] < 1500)
    {
        flymode = LOCKED;
        MOTOR_Locked();
    }

    OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR            */
}
