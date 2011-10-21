#include "system.h"

System::System() {
    armed = false;
    motor[MT].attach(PMT);
    motor[MR].attach(PMR);
    motor[ML].attach(PML);
    tailServo.attach(PST);

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    motor[0].write(0);
    motor[1].write(0);
    motor[2].write(0);

    motorVal[0] = 0;
    motorVal[1] = 0;
    motorVal[2] = 0;
    
    myIMU.Init();
}

void System::UpdateHoverPos(int axisValues[4]) {
    // TODO: Get DCM output from myIMU and do something here to calculate a reasonable target DCM based on axisValues[].
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            targetDCM[i][j] = 0;   // TODO: UpdateHoverPos should run a PID loop and determine targetDCM based on current DCM position reported by myIMU.
        }
    }
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
        #ifdef REPORT_MOTORVAL
        Serial.print("_ ");
        #endif
        for (int i=0; i<3; i++) {
            motor[i].write(TMIN);
            #ifdef REPORT_MOTORVAL
            Serial.print(motorVal[i]);
            Serial.print(" ");
            #endif
        }
        tailServo.write(90);
        #ifdef REPORT_MOTORVAL
        Serial.print(tailServoVal);
        #endif
        if (motorVal[MT] == TMIN && motorVal[MR] == TMIN && motorVal[ML] == TMIN) {
            armed = true;
            Serial.println("System: Motors armed.");
        }
    }
    else {
        #ifdef REPORT_MOTORVAL
        Serial.print("! ");
        #endif

        // motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs .ints

        for (int i=0; i<3; i++) {
            motor[i].write(motorVal[i]);   // Write motor values to motors.
            #ifdef REPORT_MOTORVAL
            Serial.print(motorVal[i]);
            Serial.print(" ");
            #endif
        }
        tailServo.write(tailServoVal);
        #ifdef REPORT_MOTORVAL
        Serial.print(tailServoVal);
        #endif
    }
}

// void System::Die() {
//     if (!armed) {
//         for (int i=0; i<3; i++) {
//             motor[i].write(TMIN);   // Send some value that is not minimum throttle.
//         }
//     }
//     else {
//         for (int i=0; i<3; i++) {
//             motorVal[i] = TMIN;   // This is inelegant. Is there a way to avoid running both of these?
//             motor[i].write(TMIN);
//         }
//         tailServo.write(90);
//         #ifdef DEBUG
//         Serial.println("System dead!");
//         #endif
//     }
// }

void System::SetMotor(int mNum, int mVal) {
    motorVal[mNum] = mVal;
}

void System::SetServo(int sVal) {
    tailServoVal = sVal;
}

