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
#include "pid.h"
#include "sensor.h"
#include "usart.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

/* Variables -----------------------------------------------------------------*/
void *ParamList[200]; // param list stores param pointer

/**
 * @brief Init param list and enable usart receive interrupt
 */
void ANO_Init(void)
{
    ParamList[11] = &PID_rate_roll.kp;
    ParamList[12] = &PID_rate_roll.ki;
    ParamList[13] = &PID_rate_roll.kd;

    ParamList[14] = &PID_rate_pitch.kp;
    ParamList[15] = &PID_rate_pitch.ki;
    ParamList[16] = &PID_rate_pitch.kd;

    ParamList[17] = &PID_rate_yaw.kp;
    ParamList[18] = &PID_rate_yaw.ki;
    ParamList[19] = &PID_rate_yaw.kd;

    ParamList[20] = &PID_angle_roll.kp;
    ParamList[21] = &PID_angle_roll.ki;
    ParamList[22] = &PID_angle_roll.kd;

    ParamList[23] = &PID_angle_pitch.kp;
    ParamList[24] = &PID_angle_pitch.ki;
    ParamList[25] = &PID_angle_pitch.kd;

    ParamList[32] = &PID_speed_altitude.kp;
    ParamList[33] = &PID_speed_altitude.ki;
    ParamList[34] = &PID_speed_altitude.kd;

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

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
 * @param id id of data frame received
 * @param sc sumcheck of data frame received
 * @param ac addcheck of data frame received
 */
static inline void ANO_Send_Verification(uint8_t id, uint8_t sc, uint8_t ac)
{
    static uint8_t data[9];
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x00;
    data[cnt++] = 3;

    data[cnt++] = id;
    data[cnt++] = sc;
    data[cnt++] = ac;

    ano_addcheck(data);

    HAL_UART_Transmit(&huart2, data, 9, 100);
}

/**
 * @brief Send Sensor data including accelerometer and gyroscope
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
 * @brief Send Sensor data magnetometer altitude and temperature
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

    int16_t temp = tmp * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = 0;
    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief Send Status in the form of euler angle
 */
uint8_t ANO_Send_Euler(uint8_t *data, float angle_rol, float angle_pit, float angle_yaw)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x03;
    data[cnt++] = 7;

    int16_t temp;
    temp = angle_rol * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = angle_pit * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = angle_yaw * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    data[cnt++] = 0;

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief Send flight mode(0 stand for locked, 1 stand for unlocked)
 */
uint8_t ANO_Send_Mode(uint8_t *data, uint8_t mode, uint8_t lock)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x06;
    data[cnt++] = 5;

    data[cnt++] = mode;
    data[cnt++] = lock;

    ano_addcheck(data);

    return cnt + 5;
}

/**
 * @brief Send target euler angle
 */
uint8_t ANO_Send_Target(uint8_t *data, float target_rol, float target_pit, float target_yaw)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x0A;
    data[cnt++] = 6;

    int16_t temp;
    temp = target_rol * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = target_pit * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = target_yaw * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief  Send tha battery's voltage and current
 */
uint8_t ANO_Send_Battery(uint8_t *data, float voltage, float current)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x0D;
    data[cnt++] = 4;

    uint16_t temp;
    temp = voltage * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);
    temp = current * 100.f;
    data[cnt++] = BYTE0(temp);
    data[cnt++] = BYTE1(temp);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief Send PWM value of four motors
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
 * @brief the control value computed by PID controller
 */
uint8_t ANO_Send_Control(uint8_t *data, int16_t roll, int16_t pitch, int16_t thrust, int16_t yaw)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x21;
    data[cnt++] = 8;

    data[cnt++] = BYTE0(roll);
    data[cnt++] = BYTE1(roll);

    data[cnt++] = BYTE0(pitch);
    data[cnt++] = BYTE1(pitch);

    data[cnt++] = BYTE0(thrust);
    data[cnt++] = BYTE1(thrust);

    data[cnt++] = BYTE0(yaw);
    data[cnt++] = BYTE1(yaw);

    ano_addcheck(data);

    return cnt + 2;
}

/**
 * @brief Send the target angle sent by remote control
 */
uint8_t ANO_Send_Remote(uint8_t *data, int16_t rol, int16_t pit, int16_t thr, int16_t yaw, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0x40;
    data[cnt++] = 20;

    data[cnt++] = BYTE0(rol);
    data[cnt++] = BYTE1(rol);

    data[cnt++] = BYTE0(pit);
    data[cnt++] = BYTE1(pit);

    data[cnt++] = BYTE0(thr);
    data[cnt++] = BYTE1(thr);

    data[cnt++] = BYTE0(yaw);
    data[cnt++] = BYTE1(yaw);

    data[cnt++] = BYTE0(aux1);
    data[cnt++] = BYTE1(aux1);

    data[cnt++] = BYTE0(aux2);
    data[cnt++] = BYTE1(aux2);

    data[cnt++] = BYTE0(aux3);
    data[cnt++] = BYTE1(aux3);

    data[cnt++] = BYTE0(aux4);
    data[cnt++] = BYTE1(aux4);

    data[cnt++] = BYTE0(aux5);
    data[cnt++] = BYTE1(aux5);

    data[cnt++] = BYTE0(aux6);
    data[cnt++] = BYTE1(aux6);

    ano_addcheck(data);
    return cnt + 2;
}

/**
 * @brief Send string
 * @param string string to send
 * @param size size of string
 * @param color BLACK RED GREEN
 */
void ANO_Send_String(char *string, size_t size, uint8_t color)
{
    uint8_t data[size + 7];
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
 * @brief Send parameter value
 * @param id parameter id
 */
static inline void ANO_Send_Param(uint16_t id)
{
    static uint8_t data[12];
    uint8_t cnt = 0;

    data[cnt++] = 0xAA;
    data[cnt++] = 0xAF;
    data[cnt++] = 0xE2;
    data[cnt++] = 6;

    data[cnt++] = BYTE0(id);
    data[cnt++] = BYTE1(id);

    int32_t val = 5;
    if (id > 10 && id < 35)
        val = (*(float *)ParamList[id]) * 100;

    data[cnt++] = BYTE0(val);
    data[cnt++] = BYTE1(val);
    data[cnt++] = BYTE2(val);
    data[cnt++] = BYTE3(val);

    ano_addcheck(data);

    HAL_UART_Transmit(&huart2, data, 12, 100);
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
    uint16_t par_id = (uint16_t)(data[5] << 8 | data[4]);

    if (id == 0xE1)
    {
        ANO_Send_Param(par_id);
    }
    else if (id == 0xE2)
    {
        if (par_id > 10 && par_id < 35)
        {
            int32_t par_val = (int32_t)(data[9] << 24 | data[8] << 16 | data[7] << 8 | data[6]);
            *((float *)ParamList[par_id]) = (float)par_val / 100.f;
            PID_Init();
        }
        ANO_Send_Verification(id, data[10], data[11]);
    }
    else if (id == 0xE0)
    {
        ANO_Send_Verification(id, data[15], data[16]);

        if (data[4] == 0x01 && data[5] == 0x00 && data[6] == 0x04)
            HMC_Calibration(magBias.array, magScale.array);
    }
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
