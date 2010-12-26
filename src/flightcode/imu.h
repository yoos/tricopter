#ifndef IMU_H
#define IMU_H

#include "itg3200.h"
#include "bma180.h"

class IMU {
    BMA180 myAcc;
    ITG3200 myGyr;
    int lastTime;
    float curPos[3]; // Array of X, Y, and Z coordinates relative to start position
    float curRot[3]; // Array of X, Y, and Z rotational angles relative to start orientation

public:
    IMU();
    void deadReckoning();
    void reset();
};

#endif

