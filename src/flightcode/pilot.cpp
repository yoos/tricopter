#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    okayToFly = false;
    
    // Assume serial inputs, all axes zeroed.
    serInput[SX] = 126;
    serInput[SY] = 126;
    serInput[ST] = 126;
    /* Keep Z value at some non-zero value (albeit very low so the tricopter 
     * doesn't fly off if something goes awry) so user is forced to adjust 
     * throttle before motors arm. */
    serInput[SZ] = 3;   

    #ifdef DEBUG
    Serial.println("Pilot here!");
    #endif

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            targetDCM[i][j] = currentDCM[i][j];
        }
    }

    numGoodComm = 0;   // Number of good communication packets.
    numBadComm = 0;   // Number of bad communication packets.
}

// int Pilot::Send(char sStr[]) {
//     zeroStr(commStr);
//     for (int i=0; i<128; i++) {
//         commStr[i] = sStr[i];
//     }
//    
//     Serial.println(commStr);
// 
//     return 0;
// }
// 
// char* Pilot::Read(char sStr[]) {
//     zeroStr(commStr);
//     int i = 0;
//     while (Serial.available() > 0) {
//         commStr[i] = sStr[i];
//         i++;
//     }
// 
//     return commStr;
// }

void Pilot::Listen() {
    if (Serial.available()) {
        serRead = Serial.read();

        // Need delay to prevent dropped bits. 500 microseconds is about as low
        // as it will go.
        delayMicroseconds(500);
        // Serial.println(serRead);

        // if (serRead == DOGBONE) {   // Receive dogbone.
        //     hasFood = true;
        // }
        if (serRead == SERHEAD) {   // Receive header.
            hasFood = true;   // Prepare food for watchdog.
            for (int i=0; i<PACKETSIZE; i++) {
                serRead = Serial.read();
                // Serial.print(int(serRead));
                // Serial.print("   ");
                delayMicroseconds(500);   // Delay to prevent dropped bits.

                // if (serRead == SERHEAD) {   // Dropped byte?
                //     // i = -1;   // Discard and start over.
                //     // Serial.println("Pilot detected packet drop.");
                //     Serial.flush();
                //     okayToFly = false;
                // }
                // else if (serRead == -1) {   // This happens when serial is empty.
                //     // Serial.println("Pilot detected malformed packet.");
                //     okayToFly = false;
                // }
                if (serRead >= INPUT_MIN && serRead <= INPUT_MAX) {
                    serInput[i] = serRead;
                    // Serial.println("Pilot determined motor value.");
                    okayToFly = true;
                    numGoodComm++;
                }
                else {
                    i = 10;
                    okayToFly = false;
                    numBadComm++;
                    Serial.print("Bad!");
                    // Flush remaining buffer to avoid taking in the wrong values.
                    Serial.flush();
                }
            }
            // Serial.println("");
        }
        else {
            // #ifdef DEBUG
            // Serial.print("Weird header!   ");   // Warn if something weird happens.
            // Serial.print(int(serRead));
            // Serial.println("");
            // #endif
            okayToFly = false;
        }
        // Serial.flush();
    }
}

void Pilot::Fly() {
    if (okayToFly) {   // Update axisVal only if okayToFly is true.
        // mySystem.UpdateHoverPos(axisVal); TODO: Implement this later.

        /* Shift serial input values [1, 251] to correct range for each axis. Z 
         * stays positive for ease of calculation. */
        axisVal[SX] = serInput[SX] - 126;   // [-125, 125]
        axisVal[SY] = serInput[SY] - 126;   // [-125, 125]
        axisVal[ST] = serInput[ST] - 126;   // [-125, 125]
        axisVal[SZ] = serInput[SZ] - 1;     // [0, 250]

        // for (int i=0; i<4; i++) {
        //     Serial.print(serInput[i]);
        //     Serial.print("   ");
        // }
        // Serial.println("");

        for (int i=0; i<2; i++) {   // First two rows of targetDCM and currentDCM should be identical until I figure out how to use all of currentDCM.
            for (int j=0; j<3; j++) {
                targetDCM[i][j] = currentDCM[i][j];
            }
        }

        targetDCM[2][0] = -axisVal[SX]*cos(PI/4)/125; //map(axisVal[SX], -125, 125, -cos(PI/4), cos(PI/4));
        targetDCM[2][1] = -axisVal[SY]*cos(PI/4)/125; //map(axisVal[SY], -125, 125, -cos(PI/4), cos(PI/4));
        targetDCM[2][2] = sqrt(1 - pow(targetDCM[2][0],2) - pow(targetDCM[2][1],2));
        //Serial.print("(");
        //Serial.print(targetDCM[2][0]);
        //Serial.print("  ");
        //Serial.print(targetDCM[2][1]);
        //Serial.print("  ");
        //Serial.print(targetDCM[2][2]);
        //Serial.print(")");

        // dir = atan2(axisVal[SY], axisVal[SX]);   // May need this eventually for IMU.

        // Intermediate calculations TODO: Move this to system code.
        motorVal[MT] = MOTOR_T_OFFSET + axisVal[SZ] - 0.6667*DCM_COEFF*(targetDCM[2][1]-currentDCM[2][1]);   // Watch out for floats vs. ints
        motorVal[MR] = MOTOR_R_OFFSET + axisVal[SZ] + 0.3333*DCM_COEFF*(targetDCM[2][1]-currentDCM[2][1]) + DCM_COEFF*(targetDCM[2][0]-currentDCM[2][0])/sqrt(3);
        motorVal[ML] = MOTOR_L_OFFSET + axisVal[SZ] + 0.3333*DCM_COEFF*(targetDCM[2][1]-currentDCM[2][1]) - DCM_COEFF*(targetDCM[2][0]-currentDCM[2][0])/sqrt(3);

        //Serial.print("(");
        //Serial.print(motorVal[MT]);
        //Serial.print("  ");
        //Serial.print(motorVal[MR]);
        //Serial.print("  ");
        //Serial.print(motorVal[ML]);
        //Serial.print(")");

        // Find max/min motor values
        mapUpper = motorVal[MT] > motorVal[MR] ? motorVal[MT] : motorVal[MR];
        mapUpper = mapUpper > motorVal[ML] ? mapUpper : motorVal[ML];
        mapLower = motorVal[MT] < motorVal[MR] ? motorVal[MT] : motorVal[MR];
        mapLower = mapLower < motorVal[ML] ? mapLower : motorVal[ML];

        // Find map boundaries (need to limit, but NOT fit, to [0, 250])
        mapUpper = mapUpper > 250 ? mapUpper : 250;
        mapLower = mapLower < 0 ? mapLower : 0;

        // Final calculations
        for (int i=0; i<3; i++) {
            motorVal[i] = map(motorVal[i], mapLower, mapUpper, TMIN, TMAX);
        }
        tailServoVal = TAIL_SERVO_DEFAULT_POSITION - 0.5*axisVal[ST];

        //Serial.print("(");
        //Serial.print(motorVal[MT]);
        //Serial.print("  ");
        //Serial.print(motorVal[MR]);
        //Serial.print("  ");
        //Serial.print(motorVal[ML]);
        //Serial.print(")");

        // Serial.print(millis());
        // Serial.print(": ");

        okayToFly = false;
    }
    else {
        // Serial.println("Pilot not okay to fly.");
    }

    #ifdef DEBUG
    Serial.println("Pilot is flying.");
    #endif
}

void Pilot::Abort() {
    /* When communication is lost, pilot should set all motorVal[] to TMIN and 
     * tell system to set all motors to TMIN. Pilot should also tell system to 
     * set armed status to false so that if communication resumes, throttle 
     * must be reduced to zero before control is restored. */
    // for (int i=0; i<3; i++) {
    //     motorVal[i] = 0;
    //     mySystem.SetMotor(i, 0);
    //     mySystem.armed = false;
    // }
    // mySystem.SetServo(90);
    #ifdef DEBUG
    Serial.println("Pilot ejected!");
    #endif
}

