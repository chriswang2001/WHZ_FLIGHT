/**
 * @file sensor.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief Driver for sensor GY-86 including MPU-6050 HMC5883L MS5611 through I2C
 * @version 1.0
 * @date 2021-11-16
 * @copyright Copyright (c) 2021
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "adc.h"
#include "ahrs.h"
#include "ano.h"
#include "arm_math.h"
#include "filter.h"
#include "i2c.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>

/* Defines -------------------------------------------------------------------*/
// #define MAG_ENABLE
#define MPU_TEMP

/* Variables -----------------------------------------------------------------*/
// full scale chose
uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_8G;
uint8_t Mscale = MFS_130Ga;

// scale resolutions per LSB for the sensors
const float aRes[4] = {2.f / 32768.f, 4.f / 32768.f, 8.f / 32768.f, 16.f / 32768.f};
const float gRes[4] = {250.f / 32768.f, 500.f / 32768.f, 1000.f / 32768.f, 2000.f / 32768.f};
const float mRes[8] = {1.f / 1.37f, 1.f / 1.09f, 1.f / 0.82f, 1.f / 0.66f, 1.f / 0.44f, 1.f / 0.39f, 1.f / 0.33f, 1.f / 0.23f};

// unit conversion
const float DegreeToRadian = PI / 180.f;
const float RadianToDegree = 180.f / PI;
const float ADCToVoltage = 3.3f / 4095.f;

FloatVector3 accel, accelBias;                                    // Stores the real sensor value in g's
FloatVector3 gyro, gyroBias;                                      // Stores the real sensor value in degrees per second
FloatVector3 mag, magBias, magScale = {.array = {1.f, 1.f, 1.f}}; // Stores the real sensor value in milliGauss
float altitude, initialAltitude;
float voltage, temperature;
uint16_t msPROM[8];

/**
 * @brief write a byte to targer sensor by I2C bus
 * @param addr address of target sensor
 * @param data data to write
 * @return true write successfully
 * @return false write failed
 */
static bool SENSOR_WriteByte(uint8_t addr, uint8_t data)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &data, 1, I2Cx_TIMEOUT))
    {
        I2C_Reset();
        return false;
    }

    return true;
}

/**
 * @brief write a byte to targer sensor's regsiter by I2C bus
 * @param addr address of target sensor
 * @param reg address of target regsiter
 * @param data data to write
 * @return true write successfully
 * @return false write failed
 */
static bool SENSOR_RegWriteByte(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, I2Cx_TIMEOUT))
    {
        I2C_Reset();
        return false;
    }

    return true;
}

/**
 * @brief read bytes from targer sensor by I2C bus
 * @param addr address of target sensor
 * @param length the length of data to read
 * @param data the pointer of data to read
 * @return true read successfully
 * @return false read failed
 */
static bool SENSOR_ReadBytes(uint8_t addr, uint8_t length, uint8_t *data)
{

    if (HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, length, I2Cx_TIMEOUT))
    {
        I2C_Reset();
        return false;
    }

    return true;
}

/**
 * @brief read bytes from targer sensor's regsiter by I2C bus
 * @param addr address of target sensor
 * @param reg address of target regsiter
 * @param length the length of data to read
 * @param data the pointer of data to read
 * @return true read successfully
 * @return false read failed
 */
static bool SENSOR_RegReadBytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
    if (HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2Cx_TIMEOUT))
    {
        I2C_Reset();
        return false;
    }

    return true;
}

/**
 * @brief read raw accelerometer data
 * @param dest destination array to store data
 */
void MPU_ReadAccelRaw(int16_t *dest)
{
    static uint8_t rawData[6];                                                  // x/y/z accel register data stored here
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    dest[0] = (int16_t)((rawData[0] << 8) | rawData[1]);                        // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    dest[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

/**
 * @brief read raw gyroscope data
 * @param dest destination array to store data
 */
void MPU_ReadGyroRaw(int16_t *dest)
{
    static uint8_t rawData[6];                                                 // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    dest[0] = (int16_t)((rawData[0] << 8) | rawData[1]);                       // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    dest[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

/**
 * @brief read raw temprature data
 * @return int16_t return temprature
 */
int16_t MPU_ReadTemp()
{
    static uint8_t rawData[2];                                                // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
    return (int16_t)((rawData[0] << 8) | rawData[1]);                         // Turn the MSB and LSB into a 16-bit value
}

/**
 * @brief read raw accelerometer temperature gyroscope data
 * @param dest destination array to store data
 */
void MPU_ReadAllRaw(int16_t *accel, int16_t *temp, int16_t *gyro)
{
    static uint8_t rawData[14];
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, &rawData[0]);
    accel[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

    temp[0] = (int16_t)((rawData[6] << 8) | rawData[7]);

    gyro[0] = (int16_t)((rawData[8] << 8) | rawData[9]);
    gyro[1] = (int16_t)((rawData[10] << 8) | rawData[11]);
    gyro[2] = (int16_t)((rawData[12] << 8) | rawData[13]);
}

/**
 * @brief read raw gyroscope data
 * @param dest destination array to store data
 */
void HMC_ReadMagRaw(int16_t *dest)
{
    static uint8_t rawData[6];                                               // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    dest[0] = ((int16_t)rawData[0] << 8) | rawData[1];                       // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
    dest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
}

/**
 * @brief read prom data
 * @param dest destination array to store data
 */
void MS_ReadPROM(uint16_t *dest)
{
    for (int i = 0; i < 8; i++)
    {
        uint8_t temp[2] = {0};
        SENSOR_WriteByte(MS5611_ADDRESS, MS5611_CMD_READ_PROM | i << 1);
        HAL_Delay(5);
        SENSOR_ReadBytes(MS5611_ADDRESS, 2, temp);
        dest[i] = (uint16_t)((temp[0] << 8) | temp[1]);
    }
}

/**
 * @brief send cmd to ms5611
 * @param cmd command
 * @return true send successfully
 * @return false send fail
 */
bool MS_WriteCMD(uint8_t cmd)
{
    return SENSOR_WriteByte(MS5611_ADDRESS, cmd);
}

/**
 * @brief read adc value from ms5611
 * @return uint32_t
 */
uint32_t MS_ReadADC(void)
{
    static uint8_t rawData[3];

    SENSOR_WriteByte(MS5611_ADDRESS, MS5611_CMD_ADC_READ);
    SENSOR_ReadBytes(MS5611_ADDRESS, 3, rawData);
    return (uint32_t)(rawData[0] << 16 | rawData[1] << 8 | rawData[2]);
}

/**
 * @brief caculate pressure from d1, d2
 * @param D1 Digital pressure value
 * @param D2 Digital temperature value, set 0 to use temperature from mpu6050
 * @return  int32_t pressure value *100 mbar
 */
static inline uint32_t MS_CalculatePressure(uint32_t D1, uint32_t D2)
{
    int32_t dT, TEMP;
    if (D2)
    {
        dT = D2 - (uint32_t)msPROM[5] * 256;
        TEMP = 2000 + ((int64_t)dT * msPROM[6]) / 8388608;
        temperature = TEMP / 100.f;
    }
    else
    {
        TEMP = temperature * 100;
        dT = ((int64_t)TEMP - 2000) * 8388608 / msPROM[6];
    }

    int64_t OFF = (int64_t)msPROM[2] * 65536 + (int64_t)msPROM[4] * dT / 128;
    int64_t SENS = (int64_t)msPROM[1] * 32768 + (int64_t)msPROM[3] * dT / 256;

    int64_t OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000)
    {
        int64_t t = (TEMP - 2000) * (TEMP - 2000);
        OFF2 = 5 * t / 2;
        SENS2 = 5 * t / 4;
    }

    if (TEMP < -1500)
    {
        int64_t t = (TEMP + 1500) * (TEMP + 1500);
        OFF2 = OFF2 + 7 * t;
        SENS2 = SENS2 + 11 * t / 2;
    }

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    return (D1 * SENS / 2097152 - OFF) / 32768;
}

const int mscount = MS5611_ADC_4096_DELAY / FLIGHT_CYCLE_MS;
/**
 * @brief Calculate altitude from ms5611
 * @return float altitude cm from sea level
 */
float MS_CalculateAltitude(void)
{
    static uint8_t state = 0;
    static uint32_t P, D1, D2;
    static float ALT, pressurebuffer[15];
#ifdef MPU_TEMP
    switch (state)
    {
    case 0:
        MS_WriteCMD(MS5611_CMD_CONV_D1 + MS5611_CMD_ADC_4096);
        break;
    case mscount + 1:
        D1 = MS_ReadADC();
        P = slidingFilterApply(pressurebuffer, MS_CalculatePressure(D1, 0), sizeof(pressurebuffer) / sizeof(float));
        ALT = 44330.0f * 100.f * (1.0f - powf((float)P / Sea_Level_Pressure, 0.19025875190f));
#ifdef DEBUG_ENABLE
        printf("d1:%d t:%.3f a:%.3f\n", D1, temperature, ALT);
#endif
        state = -1;
        break;
    default:
        break;
    }
#else
    switch (state)
    {
    case 0:
        if (D1)
        {
            D2 = MS_ReadADC();
            P = slidingFilterApply(pressurebuffer, MS_CalculatePressure(D1, D2), sizeof(pressurebuffer) / sizeof(float));
            ALT = 44330.0f * 100.f * (1.0f - powf((float)P / Sea_Level_Pressure, 0.19025875190f));
        }
        MS_WriteCMD(MS5611_CMD_CONV_D1 + MS5611_CMD_ADC_4096);
        break;
    case mscount + 1:
        D1 = MS_ReadADC();
        MS_WriteCMD(MS5611_CMD_CONV_D2 + MS5611_CMD_ADC_4096);
        break;
    case 2 * mscount + 1:
        state = -1;
    default:
        break;
    }
#endif

    state++;
    return ALT;
}

/**
 * @brief to test if mpu is connected
 * @return true mpu is connected
 * @return false mpu is not connected
 */
bool MPU_Connect(void)
{
    uint8_t c;
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_WHO_AM_I, 1, &c); // Read WHO_AM_I register for MPU-6050
    printf("I AM %x,  I Should Be %x\n", c, MPU6050_ADDRESS);

    if (c == MPU6050_ADDRESS)
    {
        printf("MPU is online...\n");
        return true;
    }

    printf("MPU is not connected. Please check it.\n");
    return false;
}

/**
 * @brief to test if hmc is connected
 * @return true hmc is connected
 * @return false hmc is not connected
 */
bool HMC_Connect(void)
{
    uint8_t e, f, g;
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_IDA, 1, &e); // Read WHO_AM_I register A for HMC5883L
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_IDB, 1, &f); // Read WHO_AM_I register B for HMC5883L
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_IDC, 1, &g); // Read WHO_AM_I register C for HMC5883L

    printf("I AM %x %x %x,  I Should Be %x %x %x\n", e, f, g, HMC5883L_IDA_RETURN, HMC5883L_IDB_RETURN, HMC5883L_IDC_RETURN);

    if (e == HMC5883L_IDA_RETURN && f == HMC5883L_IDB_RETURN && g == HMC5883L_IDC_RETURN)
    {
        printf("HMC is online...\n");
        return true;
    }

    printf("HMC is not connected. Please check it.\n");
    return false;
}

/**
 * @brief Accelerometer and gyroscope self test; check calibration wrt factory settings
 * @return true pass self test
 * @return false fail self test
 */
bool MPU_SelfTest(void) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6];
    uint8_t selfTest[6];
    float factoryTrim[6];
    float destination[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};

    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_SELF_TEST_X, 4, rawData);

    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03);      // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)selfTest[0] - 1.0f) / 30.0f))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)selfTest[1] - 1.0f) / 30.0f))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f * 0.34f) * (powf((0.92f / 0.34f), (((float)selfTest[2] - 1.0f) / 30.0f))); // FT[Za] factory trim calculation
    factoryTrim[3] = (25.0f * 131.0f) * (powf(1.046f, ((float)selfTest[3] - 1.0f)));                     // FT[Xg] factory trim calculation
    factoryTrim[4] = (-25.0f * 131.0f) * (powf(1.046f, ((float)selfTest[4] - 1.0f)));                    // FT[Yg] factory trim calculation
    factoryTrim[5] = (25.0f * 131.0f) * (powf(1.046f, ((float)selfTest[5] - 1.0f)));                     // FT[Zg] factory trim calculation

    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10); // disable self test on all three axes and set accelerometer range to +/- 8 g
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x00);  // diaable self test on all three axes and set gyro range to +/- 250 degrees/s
    HAL_Delay(50);                                                    // Delay a while to let the device execute the self-test

    for (int ii = 0; ii < 200; ii++)
    { // get average current values of gyro and acclerometer

        SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);              // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);             // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    for (int ii = 0; ii < 3; ii++)
    { // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    HAL_Delay(50);                                                    // Delay a while to let the device execute the self-test

    for (int ii = 0; ii < 200; ii++)
    { // get average self-test values of gyro and acclerometer

        SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);            // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);           // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    for (int ii = 0; ii < 3; ii++)
    { // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10); // disable self test on all three axes and set accelerometer range to +/- 8 g
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x00);  // diaable self test on all three axes and set gyro range to +/- 250 degrees/s

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 3; i++)
    {
        destination[i] = 100.f * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.f;         // Report percent differences
        destination[i + 3] = 100.f * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.f; // Report percent differences

        if (destination[i] > 14.f || destination[i] < -14.f || destination[i + 3] > 14.f || destination[i + 3] < -14.f)
        {
            printf("MPU fail Selftest in destination[%d]:%.2f!\n", i, destination[i]);
            return false;
        }
    }

    printf("MPU pass Selftest!\n");
    return true;
}

/**
 * @brief Magnetometer self test; check calibration wrt factory settings
 * @return true pass self test
 * @return false fail self test
 */
bool HMC_SelfTest(void)
{
    int16_t selfTest[3] = {0, 0, 0};
    //  Perform self-test and calculate temperature compensation bias
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x71); // set 8-average, 15 Hz default, positive self-test measurement
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0xA0); // set gain (bits[7:5]) to 5
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);     // enable continuous data mode
    HAL_Delay(150);                                                 // wait 150 ms

    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};                                 // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];                   // Turn the MSB and LSB into a signed 16-bit value
    selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
    selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x70); // exit self test

    for (int i = 0; i < 3; i++)
    {
        if (selfTest[i] > 575 || selfTest[i] < 243)
        {
            printf("HMC fail selftest in selfTest[%d]:%d!\n", i, selfTest[i]);
            return false;
        }
    }

    printf("HMC pass selftest!\n");
    return true;
}

/**
 * @brief calculate checksum from PROM register contents
 * @param n_prom prom data
 * @return true
 * @return false
 */
bool MS_CheckCRC(uint16_t *n_prom)
{
    int cnt;               // simple counter
    unsigned int n_rem;    // crc reminder
    unsigned int crc_read; // original value of the crc
    unsigned char n_bit;
    n_rem = 0x00;
    crc_read = n_prom[7];               // save read CRC
    n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++)      // operation is performed on bytes
    {                                   // choose LSB or MSB
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
    n_prom[7] = crc_read;             // restore the crc_read to its original place

    unsigned int crc_caculate = n_rem ^ 0x00;
    crc_read &= 0x000F;
    printf("Checksum = %d, should be %d\n", crc_caculate, crc_read);
    if (crc_caculate != crc_read)
    {
        printf("MS fail CRC check!\n");
        return false;
    }

    printf("MS pass CRC check!\n");
    return true;
}

/**
 * @brief Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
 *        of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
 * @param gyrobias gyroscope bias
 * @param accbias accelerometer bias
 */
void MPU_Calibrate(float *gyrobias, float *accbias)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);
    HAL_Delay(200);

    // Configure device for bias calculation
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 0x00);   // Disable all interrupts
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_FIFO_EN, 0x00);      // Disable FIFO
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_I2C_MST_CTRL, 0x00); // Disable I2C master
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    HAL_Delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x40); // Enable FIFO
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    HAL_Delay(80);                                                 // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);         // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
    }
    else
    {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // construct gyro bias in deg/s for later manual subtraction
    gyrobias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    gyrobias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyrobias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Output scaled accelerometer biases for manual subtraction in the main program
    accbias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accbias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accbias[2] = (float)accel_bias[2] / (float)accelsensitivity;

    printf("MPU6050 bias\n");
    printf(" x\t  y\t  z  \n");
    printf("%f\t%f\t%f\t mg\n", accelBias.array[0], accelBias.array[1], accelBias.array[2]);
    printf("%f\t%f\t%f\t ??/s\n", gyroBias.array[0], gyroBias.array[1], gyroBias.array[2]);
}

/**
 * @brief caculate error from soft iron and hard iron
 * @param magbias offset error by hard iron
 * @param magscale scale error by soft iron
 */
void HMC_Calibration(float *magbias, float *magscale)
{
    int ii = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3], mag_pre[3] = {0};

    printf("Mag Calibration: Wave device in a figure eight until done!\n");
    HAL_Delay(1000);

    while (ii < 1500)
    {
        uint8_t status = 0;
        SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0X01);
        HAL_Delay(7);
        SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_STATUS, 1, &status);
        if (status & 0x01)
        {
            ii++;
            HMC_ReadMagRaw(mag_temp); // Read the mag data
            for (int jj = 0; jj < 3; jj++)
            {
                mag_temp[jj] = mag_pre[jj] * 0.2 + mag_temp[jj] * 0.8;
                mag_pre[jj] = mag_temp[jj];
                if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
            }
        }
    }

    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0X00);

    printf("mag x min: %d   max:%d\n", mag_min[0], mag_max[0]);
    printf("mag y min: %d   max:%d\n", mag_min[1], mag_max[1]);
    printf("mag z min: %d   max:%d\n", mag_min[2], mag_max[2]);

    // Get hard iron correction
    for (int i = 0; i < 3; i++)
        mag_bias[i] = (mag_max[i] + mag_min[i]) / 2; // get average x y zmag bias in counts

    for (int i = 0; i < 3; i++)
        magbias[i] = mag_bias[i] * mRes[Mscale]; // save mag biases in G for main program

    // Get soft iron correction estimate
    for (int i = 0; i < 3; i++)
        mag_scale[i] = (mag_max[i] - mag_min[i]) / 2; // get average x  y zaxis max chord length in counts

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.f;

    for (int i = 0; i < 3; i++)
        magscale[i] = avg_rad / ((float)mag_scale[i]);

    printf("HMC5883L bias and scale\n");
    printf(" x\t  y\t  z  \n");
    printf("%.1f\t%.1f\t%.1f\t bias\n", magbias[0], magbias[1], magbias[2]);
    printf("%.1f\t%.1f\t%.1f\t scale\n", magscale[0], magscale[1], magscale[2]);
}

/**
 * @brief MPU Initialize
 */
void MPU_Init(void)
{
    MPU_SelfTest();
    MPU_Calibrate(gyroBias.array, accelBias.array);

    float aRMS, gRMS;
    arm_rms_f32(accelBias.array, 3, &aRMS);
    printf("aRMS:%f\n", aRMS);
    arm_rms_f32(gyroBias.array, 3, &gRMS);
    printf("gRMS:%f\n", gRMS);

    if (aRMS > 0.03f)
    {
        printf("using prepared accelBias\n");
        accelBias.array[0] = 0.027039f;
        accelBias.array[1] = -0.007324f;
        accelBias.array[2] = 0.020325f;
    }

    if (gRMS > 1.0f)
    {
        printf("using prepared gyroBias\n");
        gyroBias.array[0] = -0.809160f;
        gyroBias.array[1] = -0.236641f;
        gyroBias.array[2] = -0.587786f;
    }

    // wake up device-don't need this here if using calibration function below
    // SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    // HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer not using filter in mpu6050 itself
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x00);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x00); // Use a 1000 Hz rate;

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c;
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &c);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &c);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

    // Enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x02);
}

/**
 * @brief HMC Initialize
 */
void HMC_Init(void)
{
    HMC_SelfTest();
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x78);        // Set magnetomer ODR to 75 Hz and number of samples averaged to 8
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, Mscale << 5); // set gain (bits[7:5]) to maximum resolution of 0.92 mG/LSB
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80);            // enable continuous data mode
}

/**
 * @brief MS Initialize
 */
void MS_Init(void)
{
    SENSOR_WriteByte(MS5611_ADDRESS, MS5611_CMD_RESET);
    HAL_Delay(100);

    MS_ReadPROM(msPROM);
    MS_CheckCRC(msPROM); // calculate checksum to ensure integrity of MS5611 calibration data

    // MS5611 needs a few readings to stabilize
    // for (int i = 0; i < 100.f * mscount; i++)
    // {
    //     temperature = (float)MPU_ReadTemp() / 340.f + 36.53f;
    //     altitude = MS_CalculateAltitude();
    //     HAL_Delay(FLIGHT_CYCLE_MS);
    // }

    // initialAltitude = altitude + 650;
    // printf("initialAltitude:%.3f\n", initialAltitude);
}

/**
 * @brief sensor initialize including mpu hmc and ms
 */
void SENSOR_Init(void)
{
    if (!MPU_Connect())
    {
        I2C_Reset();
        HAL_Delay(100);
    }

    MPU_Init();

#ifdef MAG_ENABLE
    if (HMC_Connect())
        HMC_Init();
#endif

    MS_Init();

    HAL_ADC_Start(&hadc1);
}

/**
 * @brief update sensor value including accel, gyro, mag, baro, adc
 */
void SENSOR_Update(void)
{
    static Int16Vector3 accelCount, gyroCount, magCount; // Stores the 16-bit signed accelerometer, gyro, magnetometer sensor output
    static int16_t temp, count;
    static int32_t pressure;

    MPU_ReadAllRaw(accelCount.array, &temp, gyroCount.array);
    for (int axis = 0; axis < 3; axis++)
    {
        accel.array[axis] = accelCount.array[axis] * aRes[Ascale] - accelBias.array[axis]; // Calculate the accleration value into actual g's
        accel.array[axis] = biquadFilterApply(&accelFilterLPF[axis], accel.array[axis]);

        gyro.array[axis] = gyroCount.array[axis] * gRes[Gscale] - gyroBias.array[axis]; // Calculate the gyro value into actual degrees per second
        gyro.array[axis] = biquadFilterApply(&gyroFilterLPF[axis], gyro.array[axis]);
    }

#ifdef MPU_TEMP
    static float temperaturebuffer[20];
    temperature = slidingFilterApply(temperaturebuffer, (float)temp / 340.f + 36.53f, sizeof(temperaturebuffer) / sizeof(float));
#endif

#ifdef MAG_ENABLE
    HMC_ReadMagRaw(magCount.array); // Read the x/y/z adc values
    for (int i = 0; i < 3; i++)
    {
        mag.array[i] = magCount.array[i] * mRes[Mscale] * magScale.array[i] - magBias.array[i]; // Calculate the magnetometer values in milliGauss
    }
#endif

    if (count < 3000)
    {
        count++;
        MS_CalculateAltitude();
        altitude = 0.f;
    }
    else if (count == 3000)
    {
        count++;
        initialAltitude = MS_CalculateAltitude();
    }
    else
        altitude = MS_CalculateAltitude() - initialAltitude;

#ifdef DEBUG_ENABLE
    printf("a:%.3f\n", altitude);
#endif

    static float vlotagebuffer[20];
    voltage = slidingFilterApply(vlotagebuffer, HAL_ADC_GetValue(&hadc1) * ADCToVoltage * 11.f, sizeof(vlotagebuffer) / sizeof(float));
    HAL_ADC_Start(&hadc1);
}
