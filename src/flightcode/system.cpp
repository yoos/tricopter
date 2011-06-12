#include "system.h"

System::System() {
    armed = false;
    motor[MT].attach(PMT);
    motor[MR].attach(PMR);
    motor[ML].attach(PML);
    tailServo.attach(PST);
}

void System::Run() {
    myIMU.Update();
    /* Don't run system unless armed!
     * Pilot will monitor serial inputs and update System::motorVal[]. System 
     * will send ESCs a "nonsense" value of 0 until it sees that all three 
     * motor values are zerod. It will then consider itself armed and start 
     * sending proper motor values.
     */
    if (!armed) {
        // Serial.println("System: Motors not armed.");
        for (int i=0; i<3; i++) {
            motor[i].write(0);   // Send some value that is not minimum throttle.
            Serial.print(motorVal[i]);
            Serial.print("   ");
        }
        tailServo.write(90);
        Serial.println(tailServoVal);
        if (motorVal[MT] == TMIN && motorVal[MR] == TMIN && motorVal[ML] == TMIN) {
            armed = true;
            Serial.println("System: Motors armed.");
        }
    }
    else {
        for (int i=0; i<3; i++) {
            motor[i].write(motorVal[i]);   // Write motor values to motors.
            Serial.print(motorVal[i]);
            Serial.print("   ");
        }
        tailServo.write(tailServoVal);
        Serial.println(tailServoVal);
        #ifdef DEBUG
        Serial.println("System wrote to motors.");
        #endif
    }
}

void System::Die() {
    if (!armed) {
        for (int i=0; i<3; i++) {
            motor[i].write(TMIN);   // Send some value that is not minimum throttle.
        }
    }
    else {
        for (int i=0; i<3; i++) {
            motorVal[i] = TMIN;   // This is inelegant. Is there a way to avoid running both of these?
            motor[i].write(TMIN);
        }
        tailServo.write(90);
        #ifdef DEBUG
        Serial.println("System dead!");
        #endif
    }
}

void System::SetMotor(int mNum, int mVal) {
    motorVal[mNum] = mVal;
}

void System::SetServo(int sVal) {
    tailServoVal = sVal;
}

