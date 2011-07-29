#ifndef IMU_H
#define IMU_H

#include "itg3200.cpp"
#include "bma180.cpp"

// Axis numbers
#define AX 0
#define AY 1
#define AZ 2
#define GX 0
#define GY 1
#define GZ 2

class IMU {
    BMA180 myAcc;
    ITG3200 myGyr;

    float aVec[3];   // Acceleration vector.
    float gVec[3];   // Gyro vector.
    float angle;

    int lastTime;
    float curPos[3]; //   Array of X, Y, and Z coordinates relative to start position
    float curRot[3]; //   Array of X, Y, and Z rotational angles relative to start orientation

    float DCM[3][3];   // Direction cosine matrix.
    float dMat[3][3];   // System update matrix.
    float tmpMat[3][3];   // Temporary matrix.

public:
    IMU();
    void Init();
    void Update();
    void Get();
    void deadReckoning();
    void reset();
};

#endif

