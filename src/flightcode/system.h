#ifndef SYSTEM_H
#define SYSTEM_H

#include <Servo.h>
#include "imu.cpp"

#define REPORT_MOTORVAL

class System {
    IMU myIMU;
    Servo motor[3];
    Servo tailServo;
    int motorVal[3];
    int tailServoVal;

public:
    bool armed;
    System();
    void Run();
    // void Die();
    void SetMotor(int, int);
    void SetServo(int);
};



#endif

