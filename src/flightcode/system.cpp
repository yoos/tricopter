#include "system.h"

System::System() {
    armed = false;
    for (int i=0; i<3; i++) {
        motor[i].write(0);   // Send some value that is not minimum throttle.
    }
}

void System::Run() {
    if (!armed) {
        #ifdef DEBUG
        Serial.println("System: Motors not armed.");
        #endif
        if (motorVal[0] == 0 && motorVal[1] == 0 && motorVal[2] == 0) {
            armed = true;
            #ifdef DEBUG
            Serial.println("System: Motors armed.");
            #endif
        }
    }
    else {
        for (int i=0; i<3; i++) {
            motor[i].write(motorVal[i]);   // Write motor values to motors.
        }
        #ifdef DEBUG
        Serial.println("System wrote to motors.");
        #endif
    }
}

void System::Die() {
    for (int i=0; i<3; i++) {
        motorVal[i] = THROTTLE_MIN;   // This is inelegant. Is there a way to avoid running both of these?
        motor[i].write(THROTTLE_MIN);
    }
    #ifdef DEBUG
    Serial.println("System dead!");
    #endif
}



