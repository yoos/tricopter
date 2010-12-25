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





