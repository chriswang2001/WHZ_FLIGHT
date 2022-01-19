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
#include "filter.h"
#include "sensor.h"

/* Defines -------------------------------------------------------------------*/
#define GRAVITY 980.665f // Standard gravitational constant
#define Q_accel 0.0003f
#define R_altitude 0.001f

/* Variables -----------------------------------------------------------------*/
FloatVector3 pos, vel;                // position and velocity estimator
FloatVector3 accelEarth;              // accel in earth coordinate
float rMat[3][3];                     // rotation matrix
float P[2][2] = {1.f, 0.f, 0.f, 1.f}; // Predicted covariance matrix 'P'

/**
 * @brief body coordinate system to earth coordinate system
 * @param v vetor to transform
 */
static void BodyToEarth_Transform(FloatVector3 *v)
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

    float x = rMat[0][0] * v->axis.x + rMat[1][0] * v->axis.y + rMat[2][0] * v->axis.z;
    float y = rMat[0][1] * v->axis.x + rMat[1][1] * v->axis.y + rMat[2][1] * v->axis.z;
    float z = rMat[0][2] * v->axis.x + rMat[1][2] * v->axis.y + rMat[2][2] * v->axis.z;

    v->axis.x = x;
    v->axis.y = y;
    v->axis.z = z;
}

/**
 * @brief caculate accel cm/s2 in earth coordinate system
 * @param a NED accel
 */
static void AccelEarth_Cal(float ax, float ay, float az)
{
    FloatVector3 accelCMSS; // accel in cm/s2

    accelCMSS.axis.x = ax * GRAVITY;
    accelCMSS.axis.y = ay * GRAVITY;
    accelCMSS.axis.z = az * GRAVITY;
    BodyToEarth_Transform(&accelCMSS);

    accelCMSS.axis.z -= GRAVITY; //去除重力
    for (int axis = 0; axis < 3; axis++)
    {
        accelCMSS.array[axis] = deadBandApply(accelCMSS.array[axis], 8);
        accelEarth.array[axis] += (accelCMSS.array[axis] - accelEarth.array[axis]) * 0.3f; //一阶低通
    }
}

/**
 * @brief position estimator using kalman filter
 */
void POS_Update()
{
    AccelEarth_Cal(-accel.axis.y, -accel.axis.x, accel.axis.z);

    float dt = deltaTick / 1000.f;
    float _dtdt = dt * dt;

    pos.axis.z += vel.axis.z * dt + 0.5f * accelEarth.axis.z * _dtdt;
    vel.axis.z += accelEarth.axis.z * dt;

    float _Q_accel_dtdt = Q_accel * _dtdt;
    P[0][0] = P[0][0] + (P[1][0] + P[0][1] + (P[1][1] + 0.25f * _Q_accel_dtdt) * dt) * dt;
    P[0][1] = P[0][1] + (P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    P[1][0] = P[1][0] + (P[1][1] + 0.5f * _Q_accel_dtdt) * dt;
    P[1][1] = P[1][1] + _Q_accel_dtdt;

    float y = altitude - pos.axis.z;

    float Sinv = 1.0f / (P[0][0] + R_altitude);

    float K[2] = {P[0][0] * Sinv, P[1][0] * Sinv};

    pos.axis.z += K[0] * y;
    vel.axis.z += K[1] * y;

    P[0][0] = P[0][0] - K[0] * P[0][0];
    P[0][1] = P[0][1] - K[0] * P[0][1];
    P[1][0] = P[1][0] - (K[1] * P[0][0]);
    P[1][1] = P[1][1] - (K[1] * P[0][1]);
}
