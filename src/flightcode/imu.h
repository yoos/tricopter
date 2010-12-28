#ifndef IMU_H
#define IMU_H

#include "itg3200.cpp"
#include "bma180.cpp"

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
    void Update();
    void Get();
    void deadReckoning();
    void reset();
};

#endif

