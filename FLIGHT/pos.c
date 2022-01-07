/**
 * @file pos.c
 * @author Chris Wang (wang20011029@foxmail.com)
 * @brief position and velocity estimator
 * @version 1.0
 * @date 2022-01-06
 * @copyright Copyright (c) 2022
 */

/* Includes ------------------------------------------------------------------*/
#include "pos.h"
#include "ahrs.h"
#include "sensor.h"

/* Defines -------------------------------------------------------------------*/
#define BARO_W 0.28f     // Correction weight for barometer
#define GRAVITY 9.80665f // Standard gravitational constant

/* Variables -----------------------------------------------------------------*/
FloatVector3 pos, vel;                          // position and velocity estimator
FloatVector3 accelEarth;                        // accel in earth coordinate
FloatVector3 accelEarthBias;
volatile float rMat[3][3];                      // rotation matrix
const float delta_t = FLIGHT_CYCLE_MS / 1000.f; // period in millisecond

static void AccelEarth_Cal(float ax, float ay, float az);
static void RotationMatrix_Cal();
static void BodyToEarth_Transform(FloatVector3 *v);

void POS_Update()
{
    AccelEarth_Cal(-accel.axis.y, -accel.axis.x, accel.axis.z);
    pos.axis.z += vel.axis.z * delta_t + accelEarth.axis.z * delta_t * delta_t / 2.f;
    vel.axis.z += accelEarth.axis.z * delta_t;
    float ewdt = (altitude - pos.axis.z) * BARO_W * delta_t;
    pos.axis.z += ewdt;
    vel.axis.z += BARO_W * ewdt;
}

static void AccelEarth_Cal(float ax, float ay, float az)
{
    FloatVector3 accelMSS; // accel in m/s2

    RotationMatrix_Cal();

    accelMSS.axis.x = ax * GRAVITY;
    accelMSS.axis.y = ay * GRAVITY;
    accelMSS.axis.z = az * GRAVITY;
    BodyToEarth_Transform(&accelMSS);

    accelMSS.axis.z -= 0.98f*GRAVITY; //去除重力
    for (int axis = 0; axis < 3; axis++)
    {
        accelEarth.array[axis] += (accelMSS.array[axis] - accelEarth.array[axis]) * 0.3f; //一阶低通
				accelEarth.array[axis] -= accelEarthBias.array[axis];
    }
}

static void RotationMatrix_Cal()
{
    float q0q0, q1q1, q1q2, q0q3, q1q3, q0q2, q2q2, q2q3, q0q1, q3q3;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q0q3 = q0 * q3;
    q1q3 = q1 * q3;
    q0q2 = q0 * q2;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q0q1 = q0 * q1;
    q3q3 = q3 * q3;

    rMat[0][0] = 2.f * q0q0 - 1.f + 2.f * q1q1;
    rMat[0][1] = 2.f * (q1q2 + q0q3);
    rMat[0][2] = 2.f * (q1q3 - q0q2);

    rMat[1][0] = 2.f * (q1q2 - q0q3);
    rMat[1][1] = 2.f * q0q0 - 1.f + 2.f * q2q2;
    rMat[1][2] = 2.f * (q2q3 + q0q1);

    rMat[2][0] = 2.f * (q1q3 + q0q2);
    rMat[2][1] = 2.f * (q2q3 - q0q1);
    rMat[2][2] = 2.f * q0q0 - 1.f + 2.f * q3q3;
}

static void BodyToEarth_Transform(FloatVector3 *v)
{
    float x = rMat[0][0] * v->axis.x + rMat[1][0] * v->axis.y + rMat[2][0] * v->axis.z;
    float y = rMat[0][1] * v->axis.x + rMat[1][1] * v->axis.y + rMat[2][1] * v->axis.z;
    float z = rMat[0][2] * v->axis.x + rMat[1][2] * v->axis.y + rMat[2][2] * v->axis.z;

    v->axis.x = x;
    v->axis.y = y;
    v->axis.z = z;
}