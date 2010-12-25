#include "system.h"

System::System() {
    for (int i=0; i<3; i++) {
        motorVal[i] = THROTTLE_MIN;   // Arm motors.
    }
}

void System::Run() {
    for (int i=0; i<3; i++) {
        motor[i].write(motorVal[i]);   // Write motor values to motors.
    }
}

void System::Die() {
    for (int i=0; i<3; i++) {
        motorVal[i] = THROTTLE_MIN;   // This is inelegant. Is there a way to avoid running both of these?
        motor[i].write(THROTTLE_MIN);
    }
}



