#ifndef SYSTEM_H
#define SYSTEM_H

#include <Servo.h>
#include "imu.cpp"

class System {
    IMU myIMU;
    Servo motor[3];
    bool armed;

public:
    System();
    void Run();
    void Die();
    int motorVal[3];
};



#endif

