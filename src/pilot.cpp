// ============================================================================
// pilot.cpp
// ============================================================================

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

    PID[PITCH].P = 60;
    PID[PITCH].I = 0;
    PID[PITCH].D = 0;
    PID[ROLL].P  = 60;
    PID[ROLL].I  = 0;
    PID[ROLL].D  = 0;

    #ifdef DEBUG
    spln("Pilot here!");
    #endif

    //for (int i=0; i<3; i++) {
    //    for (int j=0; j<3; j++) {
    //        targetDCM[i][j] = gyroDCM[i][j];
    //    }
    //}

    numGoodComm = 0;   // Number of good communication packets.
    numBadComm = 0;   // Number of bad communication packets.
}

void Pilot::Listen() {
    if (Serial.available()) {
        serRead = Serial.read();

        // Need delay to prevent dropped bits. 500 microseconds is about as low
        // as it will go.
        delayMicroseconds(500);

        // if (serRead == DOGBONE) {   // Receive dogbone.
        //     hasFood = true;
        // }
        if (serRead == SERHEAD) {   // Receive header.
            hasFood = true;   // Prepare food for watchdog.
            for (int i=0; i<PACKETSIZE; i++) {
                serRead = Serial.read();
                delayMicroseconds(500);   // Delay to prevent dropped bits.

                // if (serRead == SERHEAD) {   // Dropped byte?
                //     // i = -1;   // Discard and start over.
                //     // spln("Pilot detected packet drop.");
                //     Serial.flush();
                //     okayToFly = false;
                // }
                // else if (serRead == -1) {   // This happens when serial is empty.
                //     // spln("Pilot detected malformed packet.");
                //     okayToFly = false;
                // }
                if (serRead >= INPUT_MIN && serRead <= INPUT_MAX) {
                    serInput[i] = serRead;
                    // spln("Pilot determined motor value.");
                    okayToFly = true;
                    numGoodComm++;
                }
                else {
                    i = 10;
                    okayToFly = false;
                    numBadComm++;
                    sp("Bad!");
                    // Flush remaining buffer to avoid taking in the wrong values.
                    Serial.flush();
                }
            }
        }
        else {
            // #ifdef DEBUG
            // sp("Weird header! ");
            // sp((int) serRead);   // Warn if something weird happens.
            // #endif
            okayToFly = false;
        }
        // Serial.flush();
    }
}

void Pilot::Talk() {
}

void Pilot::Fly() {
    //sp("(");
    //sp(numGoodComm);
    //sp("/");
    //sp(numBadComm);
    //sp(") ");
    //if (okayToFly) {   // Update axisVal only if okayToFly is true.
    if (true) {
        // mySystem.UpdateHoverPos(axisVal); TODO: Implement this later.

        /* Shift serial input values [1, 251] to correct range for each axis. Z 
         * stays positive for ease of calculation. */
        axisVal[SX] = (float) serInput[SX] - 126;   // [-125, 125]
        axisVal[SY] = (float) serInput[SY] - 126;   // [-125, 125]
        axisVal[ST] = (float) serInput[ST] - 126;   // [-125, 125]
        axisVal[SZ] = (float) serInput[SZ] - 1;     // [0, 250]

        // Calculate target rotation vector based on joystick input scaled to a maximum rotation of PI/6.
        targetRot[0] = -(axisVal[SY]/125 * PI/6 + gyroDCM[1][2]);
        targetRot[1] =  (axisVal[SX]/125 * PI/6 + gyroDCM[0][2]);
        targetRot[2] = -(axisVal[ST]/125 * PI/6);

        // ====================================================================
        // Calculate motor/servo values.
        //     MOTOR_X_OFFSET: Offset starting motor values to account for
        //                     chassis imbalance.
        //     MOTOR_X_SCALE: Scale targetRot.
        //     axisVal[SZ]: Throttle.
        // TODO: MOTOR_X_SCALE should be replaced by actual PID gains. Besides,
        // it's incorrect to scale in the negative direction if the
        // corresponding arm is heavier.
        // ====================================================================

        // MOTORVAL SCHEME 1
        //motorVal[MT] = MOTOR_T_OFFSET + axisVal[SZ] + MOTOR_T_SCALE * (-targetRot[0]/2);
        //motorVal[MR] = MOTOR_R_OFFSET + axisVal[SZ] + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]/sqrt(3));
        //motorVal[ML] = MOTOR_L_OFFSET + axisVal[SZ] + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]/sqrt(3));

        // MOTORVAL SCHEME 2
        motorVal[MT] = MOTOR_T_OFFSET + axisVal[SZ] + MOTOR_T_SCALE * (-targetRot[0]);
        motorVal[MR] = MOTOR_R_OFFSET + axisVal[SZ] + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]);
        motorVal[ML] = MOTOR_L_OFFSET + axisVal[SZ] + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]);

        // MOTORVAL SCHEME 3
        //motorVal[MT] = MOTOR_T_OFFSET + axisVal[SZ] + MOTOR_T_SCALE * (-targetRot[0]*2);
        //motorVal[MR] = MOTOR_R_OFFSET + axisVal[SZ] + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]*sqrt(3));
        //motorVal[ML] = MOTOR_L_OFFSET + axisVal[SZ] + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]*sqrt(3));

        tailServoVal = TAIL_SERVO_DEFAULT_POSITION + TAIL_SERVO_SCALE * targetRot[2];   // TODO: Which direction is positive?

//        motorVal[MT] = MOTOR_T_SCALE * (MOTOR_T_OFFSET + axisVal[SZ] + 0.6667*(GYRO_COEFF*gVal[0] + commandPitch));   // Watch out for floats vs. ints
//        motorVal[MR] = MOTOR_R_SCALE * (MOTOR_R_OFFSET + axisVal[SZ] + 0.3333*(-GYRO_COEFF*gVal[0] - commandPitch) + (GYRO_COEFF*gVal[1] - commandRoll)/sqrt(3));
//        motorVal[ML] = MOTOR_L_SCALE * (MOTOR_L_OFFSET + axisVal[SZ] + 0.3333*(-GYRO_COEFF*gVal[0] - commandPitch) + (-GYRO_COEFF*gVal[1] + commandRoll)/sqrt(3));

        // Find min/max of the three motor values.
        maxMotorVal = motorVal[MT] > motorVal[MR] ? motorVal[MT] : motorVal[MR];
        maxMotorVal = maxMotorVal  > motorVal[ML] ? maxMotorVal  : motorVal[ML];
        minMotorVal = motorVal[MT] < motorVal[MR] ? motorVal[MT] : motorVal[MR];
        minMotorVal = minMotorVal  < motorVal[ML] ? minMotorVal  : motorVal[ML];

        // Find map boundaries (need to limit, but NOT fit, to minimum and
        // maximum throttle [TMIN, TMAX]). Doing this incorrectly will result
        // in motor values stuck mostly at either extremes.
        mapUpper = maxMotorVal < TMAX ? maxMotorVal : TMAX;
        mapLower = minMotorVal > TMIN ? minMotorVal : TMIN;

        // Remap range [minMotorVal, maxMotorVal] to [mapLower, mapUpper].
        for (int i=0; i<3; i++) {
            motorVal[i] = map(motorVal[i], minMotorVal, maxMotorVal, mapLower, mapUpper);
        }
//        tailServoVal = TAIL_SERVO_DEFAULT_POSITION - 0.5*axisVal[ST];
//        tailServoVal = map(tailServoVal, TAIL_SERVO_DEFAULT_POSITION-125*0.5, TAIL_SERVO_DEFAULT_POSITION+125*0.5, 0, TAIL_SERVO_DEFAULT_POSITION+125*0.5);

        okayToFly = false;
    }
    else {
        // spln("Pilot not okay to fly.");
    }

    #ifdef DEBUG
    spln("Pilot is flying.");
    #endif
}

void Pilot::Abort() {
    /* When communication is lost, pilot should set a bunch of stuff to safe
     * values. */
    serInput[SX] = 126;
    serInput[SY] = 126;
    serInput[ST] = 126;
    serInput[SZ] = 3;
    okayToFly = false;
    tailServoVal = 90;
    
    #ifdef DEBUG
    spln("Pilot ejected!");
    #endif
}

