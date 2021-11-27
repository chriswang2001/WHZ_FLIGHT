/**
 * @file ano.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Send and Receive data with ano v7 host
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

uint8_t data[100];

/**
 * @brief check sum_check and add_check
 * @param data pointer of received data
 * @return true when data is correct
 * @return false when data is incorrect
 */
static inline bool ano_check(uint8_t *data)
{
    uint8_t sum_check = 0;
    uint8_t add_check = 0;

    uint8_t cnt = data[3] + 4;

    for (size_t i = 0; i < cnt; i++)
    {
        sum_check += data[i];
        add_check += sum_check;
    }

    if (sum_check == data[cnt] && add_check == data[cnt + 1])
        return true;

    return false;
}

/**
 * @brief Add sum_check and add_check in data frame
 * @param data the pointer of data to send
 */
static inline void ano_addcheck(uint8_t *data)
{
    uint8_t sum_check = 0;
    uint8_t add_check = 0;

    uint8_t cnt = data[3] + 4;

    for (size_t i = 0; i < cnt; i++)
    {
        sum_check += data[i];
        add_check += sum_check;
    }

    data[cnt++] = sum_check;
    data[cnt++] = add_check;
}

/**
 * @brief Send Verification
 * @param data data to send
 * @param id id get in data frame
 * @param sc sc get in data frame
 * @param ac ac get in data frame
 * @return uint8_t the length of data frame
 */
// static inline void ANO_Send_Verification(uint8_t id, uint8_t sc, uint8_t ac)
// {
//     uint8_t cnt = 0;

//     data[cnt++] = 0xAA;
//     data[cnt++] = 0xAF;
//     data[cnt++] = 0x00;
//     data[cnt++] = 3;

//     data[cnt++] = id;
//     data[cnt++] = sc;
//     data[cnt++] = ac;

//     ano_addcheck(data);

//     return cnt + 2;
// }

/**
 * @brief  Send Sensor data including accelerometer and gyroscope
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Sensor(uint8_t *data, int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x01;
    data[cnt++] = 13;

    data[cnt++] = BYTE0(a_x);
    data[cnt++] = BYTE1(a_x);

    data[cnt++] = BYTE0(a_y);
    data[cnt++] = BYTE1(a_y);

    data[cnt++] = BYTE0(a_z);
    data[cnt++] = BYTE1(a_z);

    data[cnt++] = BYTE0(g_x);
    data[cnt++] = BYTE1(g_x);

    data[cnt++] = BYTE0(g_y);
    data[cnt++] = BYTE1(g_y);

    data[cnt++] = BYTE0(g_z);
    data[cnt++] = BYTE1(g_z);

    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send Sensor data magnetometer altitude and temperature
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Sensor2(uint8_t *data, int16_t m_x, int16_t m_y, int16_t m_z, int32_t alt, float tmp)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x02;
    data[cnt++] = 14;

    data[cnt++] = BYTE0(m_x);
    data[cnt++] = BYTE1(m_x);

    data[cnt++] = BYTE0(m_y);
    data[cnt++] = BYTE1(m_y);

    data[cnt++] = BYTE0(m_z);
    data[cnt++] = BYTE1(m_z);

    data[cnt++] = BYTE0(alt);
    data[cnt++] = BYTE1(alt);
    data[cnt++] = BYTE2(alt);
    data[cnt++] = BYTE3(alt);

    int16_t temp = (int)tmp * 10;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = 0;
    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send Status in the form of euler angle
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Euler(uint8_t *data, float angle_rol, float angle_pit, float angle_yaw)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x03;
    data[cnt++] = 7;

    uint16_t temp;
    temp = (int)angle_rol * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)angle_pit * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)angle_yaw * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send Status in the form of quaternion
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Quaternion(uint8_t *data, float q1, float q2, float q3, float q4)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x04;
    data[cnt++] = 9;

    uint16_t temp;
    temp = (int)q1 * 10000;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)q2 * 10000;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)q3 * 10000;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)q4 * 10000;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send Altitude
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Altitude(uint8_t *data, int32_t fusion, int32_t addition)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x05;
    data[cnt++] = 9;

    data[cnt++] = BYTE0(fusion);
    data[cnt++] = BYTE1(fusion);
    data[cnt++] = BYTE2(fusion);
    data[cnt++] = BYTE3(fusion);

    data[cnt++] = BYTE0(addition);
    data[cnt++] = BYTE1(addition);
    data[cnt++] = BYTE2(addition);
    data[cnt++] = BYTE3(addition);

    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send tha battery's voltage and current
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Battery(uint8_t *data, float voltage, float current)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x0D;
    data[cnt++] = 4;

    uint16_t temp;
    temp = (int)voltage * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = (int)current * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send string
 * @param data data to send
 * @retval the length of data
 */
void ANO_Send_String(char *string, size_t size, uint8_t color)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0xA0;
    data[cnt++] = size + 1;

    data[cnt++] = color;

    for (int i = 0; i < size; i++)
        data[cnt++] = string[i];

    ano_addcheck(data);

    HAL_UART_Transmit(&huart2, data, size + 7, 100);
}

/**
 * @brief  Send PWM value of four motors
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_PWM(uint8_t *data, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x20;
    data[cnt++] = 8;

    data[cnt++] = BYTE0(p1);
    data[cnt++] = BYTE1(p1);

    data[cnt++] = BYTE0(p2);
    data[cnt++] = BYTE1(p2);

    data[cnt++] = BYTE0(p3);
    data[cnt++] = BYTE1(p3);

    data[cnt++] = BYTE0(p4);
    data[cnt++] = BYTE1(p4);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  the control value computed by PID controller
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Control(uint8_t *data, float roll, float pitch, uint16_t thrust, int16_t yaw)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x41;
    data[cnt++] = 14;

    uint16_t temp;
    temp = (int)roll * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    temp = (int)pitch * 100;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = BYTE0(thrust);
    data[cnt++] = BYTE1(thrust);

    data[cnt++] = BYTE0(yaw);
    data[cnt++] = BYTE1(yaw);

    ano_addcheck(data);

    cnt += 6;
    //    HAL_UART_Transmit_DMA(&huart2, data, cnt + 2);

    return cnt + 6;
}

/**
 * @brief  Send the target angle sent by remote control
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Remote(uint8_t *data, int16_t *ch, uint8_t size)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x40;
    data[cnt++] = 20;

    for (int i = 0; i < size; i++)
    {
        data[cnt++] = BYTE0(ch[i]);
        data[cnt++] = BYTE1(ch[i]);
    }

    ano_addcheck(data);

    // some spaces are reseved
    return 26;
}

/**
 * @brief  Send Verification
 * @param data data to send
 * @retval the length of data
 */
uint8_t ANO_Send_Param(uint8_t *data, uint16_t id, int32_t val)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0xE2;
    data[cnt++] = 6;

    data[cnt++] = BYTE0(id);
    data[cnt++] = BYTE1(id);

    data[cnt++] = BYTE0(val);
    data[cnt++] = BYTE1(val);
    data[cnt++] = BYTE2(val);
    data[cnt++] = BYTE3(val);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief Analyze received data from ano host
 * @param data data frame received
 */
void ANO_Receive_Analyze(uint8_t *data)
{
    if (!ano_check(data))
        return;

    uint8_t id = data[2];

    if (id == 0XE1)
    {
        static uint8_t a[12];
        uint16_t par = (uint16_t)data[4];
        ANO_Send_Param(a, par, 5);

        HAL_UART_Transmit(&huart2, a, 12, 10);
    }
    // else if (id == 0XE2)
    // {
    // }
}

/**
 * @brief Receive data from ano host and should be called in usart irq handler
 * @param byte the byte receievd
 */
void ANO_Receive_Prepare(uint8_t byte)
{
    static uint8_t data[50], state = 0, cnt;

    if (state == 0 && byte == 0xAA)
    {
        state = 1;
        data[0] = byte;
    }
    else if (state == 1)
    {
        state = 2;
        data[1] = byte;
    }
    else if (state == 2)
    {
        state = 3;
        data[2] = byte;
    }
    else if (state == 3)
    {
        state = 4;
        data[3] = byte;
        cnt = 0;
    }
    else if (state == 4)
    {
        data[4 + cnt++] = byte;
        if (cnt >= data[3])
            state = 5;
    }
    else if (state == 5)
    {
        state = 6;
        data[4 + cnt++] = byte;
    }
    else if (state == 6)
    {
        state = 0;
        data[4 + cnt++] = byte;
        ANO_Receive_Analyze(data);
    }
    else
        state = 0;
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register             */
    OS_CPU_SR cpu_sr;
#endif

    OS_ENTER_CRITICAL();
    OSIntEnter(); /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        uint8_t ch = READ_REG(huart2.Instance->DR);
        // WRITE_REG(huart2.Instance->DR, ch);
        ANO_Receive_Prepare(ch);
    }
    else if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != RESET)
    {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);

        /* Tx process is ended, restore huart->gState to Ready */
        huart2.gState = HAL_UART_STATE_READY;
    }
    else
        HAL_UART_IRQHandler(&huart2);

    OSIntExit(); /* Tell uC/OS-II that we are leaving the ISR            */
}
