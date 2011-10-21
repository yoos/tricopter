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

    float targetDCM[3][3];   // Target position DCM sent by Pilot.

public:
    bool armed;

    System();
    void UpdateHoverPos(int*);   // Pilot sends X, Y, T, and Z inputs to system.
    void Run();   // Control motors based on pilot and IMU inputs.
    // void Die();
    void SetMotor(int, int);
    void SetServo(int);
};



#endif

