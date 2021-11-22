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
#include "i2c.h"
#include <math.h>
#include <stdio.h>

/* Variables -----------------------------------------------------------------*/
// full scale chose
uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_8G;
uint8_t Mscale = MFS_130Ga;

// scale resolutions per LSB for the sensors
float aRes[4] = {16384.f, 8192.f, 4096.f, 2048.f};
float gRes[4] = {131.0f, 62.5f, 32.8f, 16.4f};
float mRes[8] = {1.37f, 1.09f, 0.82f, 0.66f, 0.44f, 0.39f, 0.33f, 0.23f};

FloatVector3 accel, gyro, mag;             // Stores the real sensor value
FloatVector3 accelBias, gyroBias, magBias; // Bias corrections for sensor

void SENSOR_Task(void)
{
    Int16Vector3 accelCount, gyroCount, magCount; // Stores the 16-bit signed accelerometer, gyro, magnetometer sensor output

    ReadAccelRaw(accelCount.array); // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    for (int i = 0; i < 3; i++)
    {
        accel.array[i] = (float)accelCount.array[i] / aRes[Ascale] - accelBias.array[i];
    }

    ReadGyroRaw(gyroCount.array); // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    for (int i = 0; i < 3; i++)
    {
        gyro.array[i] = (float)gyroCount.array[i] / gRes[Gscale] - gyroBias.array[i];
    }

    ReadMagRaw(magCount.array); // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    for (int i = 0; i < 3; i++)
    {
        mag.array[i] = (float)magCount.array[i] / mRes[Mscale] - magBias.array[i];
    }

    // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
    // like the gyro and accelerometer biases
    // magbias[0] = +56.;  // User environmental x-axis correction in milliGauss
    // magbias[1] = -118.; // User environmental y-axis correction in milliGauss
    // magbias[2] = +35.;  // User environmental z-axis correction in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
}

void SENSOR_Init(void)
{
    if (MPU_Connect())
        MPU_Init();

    if (HMC_Connect())
        HMC_Init();
}

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
        return false;

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
        return false;

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

    if (HAL_I2C_Master_Receive(&hi2c1, addr, data, length, I2Cx_TIMEOUT))
        return false;

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
        return false;

    return true;
}

/**
 * @brief read raw accelerometer data
 * @param dest destination array to store data
 */
void ReadAccelRaw(int16_t *dest)
{
    uint8_t rawData[6];                                                         // x/y/z accel register data stored here
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    dest[0] = (int16_t)((rawData[0] << 8) | rawData[1]);                        // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    dest[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

/**
 * @brief read raw gyroscope data
 * @param dest destination array to store data
 */
void ReadGyroRaw(int16_t *dest)
{
    uint8_t rawData[6];                                                        // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    dest[0] = (int16_t)((rawData[0] << 8) | rawData[1]);                       // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    dest[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

/**
 * @brief read raw gyroscope data
 * @param dest destination array to store data
 */
void ReadMagRaw(int16_t *dest)
{
    uint8_t rawData[6];                                                      // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    dest[0] = ((int16_t)rawData[0] << 8) | rawData[1];                       // Turn the MSB and LSB into a signed 16-bit value
    dest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
    dest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
}

// /**
//  * @brief read raw temperature
//  * @return int16_t data read
//  */
// int16_t ReadTempRaw(void)
// {
//     uint8_t rawData[2];                                            // x/y/z gyro register data stored here
//     SENSOR_RegReadBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
//     return ((int16_t)rawData[0]) << 8 | rawData[1];                // Turn the MSB and LSB into a 16-bit value
// }

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

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_XG_OFFS_USRH, data[0]); // might not be supported in MPU6050
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_XG_OFFS_USRL, data[1]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_YG_OFFS_USRH, data[2]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_YG_OFFS_USRL, data[3]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ZG_OFFS_USRH, data[4]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ZG_OFFS_USRL, data[5]);

    gyrobias[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyrobias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyrobias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0};                                  // A place to hold the factory accelerometer trim biases
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_XA_OFFSET_L_TC, data[1]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_YA_OFFSET_H, data[2]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_YA_OFFSET_L_TC, data[3]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ZA_OFFSET_H, data[4]);
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    accbias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accbias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accbias[2] = (float)accel_bias[2] / (float)accelsensitivity;

    printf("MPU6050 bias\n");
    printf(" x\t  y\t  z  \n");
    printf("%d\t%d\t%d\t mg\n", (int)(1000 * accelBias.array[0]), (int)(1000 * accelBias.array[1]), (int)(1000 * accelBias.array[2]));
    printf("%.1f\t%.1f\t%.1f\t o/s\n", gyroBias.array[0], gyroBias.array[1], gyroBias.array[2]);
}

/**
 * @brief Accelerometer and gyroscope self test; check calibration wrt factory settings
 * @return true pass self test
 * @return false fail self test
 */
bool MPU_SelfTest(void) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];
    float destination[6];

    // Configure the accelerometer for self-test
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    HAL_Delay(250);                                                   // Delay a while to let the device execute the self-test

    SENSOR_RegReadBytes(MPU6050_ADDRESS, MPU6050_SELF_TEST_X, 4, rawData);
    // rawData[0] = SENSOR_ReadByte(MPU6050_ADDRESS, MPU6050_SELF_TEST_X); // X-axis self-test results
    // rawData[1] = SENSOR_ReadByte(MPU6050_ADDRESS, MPU6050_SELF_TEST_Y); // Y-axis self-test results
    // rawData[2] = SENSOR_ReadByte(MPU6050_ADDRESS, MPU6050_SELF_TEST_Z); // Z-axis self-test results
    // rawData[3] = SENSOR_ReadByte(MPU6050_ADDRESS, MPU6050_SELF_TEST_A); // Mixed-axis self-test results

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

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++)
    {
        destination[i] = 100.0f + 100.0f * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
    }

    printf("x-axis self test: acceleration trim within : ");
    printf("%.2f", destination[0]);
    printf("%% of factory value\n");
    printf("y-axis self test: acceleration trim within : ");
    printf("%.2f", destination[1]);
    printf("%% of factory value\n");
    printf("z-axis self test: acceleration trim within : ");
    printf("%.2f", destination[2]);
    printf("%% of factory value\n");
    printf("x-axis self test: gyration trim within : ");
    printf("%.2f", destination[3]);
    printf("%% of factory value\n");
    printf("y-axis self test: gyration trim within : ");
    printf("%.2f", destination[4]);
    printf("%% of factory value\n");
    printf("z-axis self test: gyration trim within : ");
    printf("%.2f", destination[5]);
    printf("%% of factory value\n");

    if (destination[0] < 1.0f && destination[1] < 1.0f && destination[2] < 1.0f && destination[3] < 1.0f && destination[4] < 1.0f && destination[5] < 1.0f)
    {
        printf("MPU pass Selftest!\n");
        return true;
    }

    return false;
}

/**
 * @brief MPU Initialize
 */
void MPU_Init(void)
{
    MPU_SelfTest();
    MPU_Calibrate(gyroBias.array, accelBias.array);

    // wake up device-don't need this here if using calibration function below
    // SENSOR_RegWriteByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    // HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    SENSOR_RegWriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; the same rate set in CONFIG above

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
 * @brief to test if hmc is connected
 * @return true hmc is connected
 * @return false hmc is not connected
 */
bool HMC_Connect()
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

bool HMC_SelfTest()
{
    int16_t selfTest[3] = {0, 0, 0};
    //  Perform self-test and calculate temperature compensation bias
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x71); // set 8-average, 15 Hz default, positive self-test measurement
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0xA0); // set gain (bits[7:5]) to 5
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80);     // enable continuous data mode
    HAL_Delay(150);                                                 // wait 150 ms

    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};                                 // x/y/z gyro register data stored here
    SENSOR_RegReadBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];                   // Turn the MSB and LSB into a signed 16-bit value
    selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
    selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x00); // exit self test

    if (selfTest[0] < 575 && selfTest[0] > 243 && selfTest[1] < 575 && selfTest[1] > 243 && selfTest[2] < 575 && selfTest[2] > 243)
    {
        printf("HMC pass selftest\n");
        return true;
    }

    return false;
}

void HMC_Init()
{
    HMC_SelfTest();
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x78);        // Set magnetomer ODR to 75 Hz and number of samples averaged to 8
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, Mscale << 5); // set gain (bits[7:5]) to maximum resolution of 0.92 mG/LSB
    SENSOR_RegWriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80);            // enable continuous data mode
}

// uint32_t ReadTempRaw(void)
// {
//     SENSOR_WriteByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + MSosr);

//     delay(ct);
//     return HAL_I2C_Master_Receive(hi2c1, ) return readRegister24(MS5611_CMD_ADC_READ);
// }

// void MS_ReadPROM(uint16_t *dest)
// {
//     for (uint8_t i = 0; i < 6; i++)
//     {
//         dest[i] = readRegister16(MS5611_CMD_READ_PROM + (i * 2));
//     }
// }

// void MS_Init()
// {
//     SENSOR_WriteByte(MS5611_ADDRESS, MS5611_CMD_RESET); // reset
//     HAL_Delay(100);

//     MS_ReadPROM(&msBias);
// }

// uint32_t MS5611::readRawPressure(void)
// {
//     Wire.beginTransmission(MS5611_ADDRESS);

// #if ARDUINO >= 100
//     Wire.write(MS5611_CMD_CONV_D1 + uosr);
// #else
//     Wire.send(MS5611_CMD_CONV_D1 + uosr);
// #endif

//     Wire.endTransmission();

//     delay(ct);

//     return readRegister24(MS5611_CMD_ADC_READ);
// }

// int32_t MS5611::readPressure(bool compensation)
// {
//     uint32_t D1 = readRawPressure();

//     uint32_t D2 = readRawTemperature();
//     int32_t dT = D2 - (uint32_t)fc[4] * 256;

//     int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
//     int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

//     if (compensation)
//     {
//         int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

//         OFF2 = 0;
//         SENS2 = 0;

//         if (TEMP < 2000)
//         {
//             OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
//             SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
//         }

//         if (TEMP < -1500)
//         {
//             OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
//             SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
//         }

//         OFF = OFF - OFF2;
//         SENS = SENS - SENS2;
//     }

//     uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

//     return P;
// }

// double MS5611::readTemperature(bool compensation)
// {
//     uint32_t D2 = readRawTemperature();
//     int32_t dT = D2 - (uint32_t)fc[4] * 256;

//     int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

//     TEMP2 = 0;

//     if (compensation)
//     {
//         if (TEMP < 2000)
//         {
//             TEMP2 = (dT * dT) / (2 << 30);
//         }
//     }

//     TEMP = TEMP - TEMP2;

//     return ((double)TEMP / 100);
// }

// // Calculate altitude from Pressure & Sea level pressure
// double MS5611::getAltitude(double pressure, double seaLevelPressure)
// {
//     return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
// }

// // Calculate sea level from Pressure given on specific altitude
// float GetSeaLevel(double pressure, double altitude)
// {
//     return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
// }

// // Read 16-bit from register (oops MSB, LSB)
// uint16_t MS5611::readRegister16(uint8_t reg)
// {
//     uint16_t value;
//     Wire.beginTransmission(MS5611_ADDRESS);
// #if ARDUINO >= 100
//     Wire.write(reg);
// #else
//     Wire.send(reg);
// #endif
//     Wire.endTransmission();

//     Wire.beginTransmission(MS5611_ADDRESS);
//     Wire.requestFrom(MS5611_ADDRESS, 2);
//     while (!Wire.available())
//     {};
// #if ARDUINO >= 100
//     uint8_t vha = Wire.read();
//     uint8_t vla = Wire.read();
// #else
//     uint8_t vha = Wire.receive();
//     uint8_t vla = Wire.receive();
// #endif;
//     Wire.endTransmission();

//     value = vha << 8 | vla;

//     return value;
// }

// // Read 24-bit from register (oops XSB, MSB, LSB)
// uint32_t MS_ReadRegister24(uint8_t reg)
// {
//     uint32_t value;
//     Wire.beginTransmission(MS5611_ADDRESS);
// #if ARDUINO >= 100
//     Wire.write(reg);
// #else
//     Wire.send(reg);
// #endif
//     Wire.endTransmission();

//     Wire.beginTransmission(MS5611_ADDRESS);
//     Wire.requestFrom(MS5611_ADDRESS, 3);
//     while (!Wire.available())
//     {};
// #if ARDUINO >= 100
//     uint8_t vxa = Wire.read();
//     uint8_t vha = Wire.read();
//     uint8_t vla = Wire.read();
// #else
//     uint8_t vxa = Wire.receive();
//     uint8_t vha = Wire.receive();
//     uint8_t vla = Wire.receive();
// #endif;
//     Wire.endTransmission();

//     value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

//     return value;
// }
