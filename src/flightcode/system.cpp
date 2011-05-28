#include "system.h"

System::System() {
    armed = false;
    motor[MT].attach(PMT);
    motor[MR].attach(PMR);
    motor[ML].attach(PML);
    tailServo.attach(PST);

    // Kill motors/servos
    for (int i=0; i<3; i++) {
        motor[i].write(0);   // Send some value that is not minimum throttle.
    }
    tailServo.write(90);
}

void System::Run() {
    myIMU.Update();
    if (!armed) {   // Don't run unless armed!
        // Serial.println("System: Motors not armed.");
        if (motorVal[MT] == TMIN && motorVal[MR] == TMIN && motorVal[ML] == TMIN) {
            armed = true;
            Serial.println("System: Motors armed.");
        }
    }
    else {
        for (int i=0; i<3; i++) {
            motor[i].write(motorVal[i]);   // Write motor values to motors.
        }
        tailServo.write(tailServoVal);
        #ifdef DEBUG
        Serial.println("System wrote to motors.");
        #endif
    }
}

void System::Die() {
    for (int i=0; i<3; i++) {
        motorVal[i] = TMIN;   // This is inelegant. Is there a way to avoid running both of these?
        motor[i].write(TMIN);
    }
    #ifdef DEBUG
    Serial.println("System dead!");
    #endif
}

void System::SetMotor(int mNum, int mVal) {
    motorVal[mNum] = mVal;
}

void System::SetServo(int sVal) {
    tailServoVal = sVal;
}

