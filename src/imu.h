// ============================================================================
// imu.h
// ============================================================================

#ifndef IMU_H
#define IMU_H

#include "itg3200.cpp"
#include "bma180.cpp"
#include "triMath.h"
#include "globals.h"

#define ACC_WEIGHT 0.020   // Accelerometer data weight relative to gyro's weight of 1

class IMU {
    BMA180 myAcc;
    ITG3200 myGyr;

    float aVec[3];   // Acceleration vector.
    float gVec[3];   // Gyro vector.
    float KB[3];     // K unity vector expressed in body coordinates.
    float wA[3];     // Corrective rotation vector based on acceleration vector.
    float wdt[3];   // Angular displacement vector = w * dt, where w is the angular velocity vector and dt is the time elapsed.
    float dDCM[3][3];   // First used to store the change in DCM to update the current DCM. Repurposed during orthonormalization to store the correction vectors for the i and j unit vectors.
    float errDCM;   // DCM error for which we need orthonormalization.

public:
    IMU();
    void Init();
    void Update();
    void Reset();
};

#endif

