#ifndef SYSTEM_H
#define SYSTEM_H

#include <Servo.h>
#include "imu.cpp"

class System {
    IMU myIMU;
    Servo motor[3];
    Servo tailServo;
    bool armed;
    int motorVal[3];

public:
    System();
    void Run();
    void Die();
    void SetMotor(int, int);
};



#endif

