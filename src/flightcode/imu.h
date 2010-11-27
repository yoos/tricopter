#ifndef IMU_H
#define IMU_H

#include "gyro.h"
#include "accelerometer.h"

class IMU {
    int lastTime;
    float curPos[3]; // Array of X, Y, and Z coordinates relative to start position
    float curRot[3]; // Array of X, Y, and Z rotational angles relative to start orientation

public:
    IMU(Gyro&, Accelerometer&);
    void deadReckoning();
    void reset();


#endif

