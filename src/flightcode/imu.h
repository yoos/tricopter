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

    float aVal[3];
    float gVal[3];
    float angle;

    int lastTime;
    float curPos[3]; // Array of X, Y, and Z coordinates relative to start position
    float curRot[3]; // Array of X, Y, and Z rotational angles relative to start orientation

public:
    IMU();
    void Init();
    void Update();
    void Get();
    void deadReckoning();
    void reset();
};

#endif

