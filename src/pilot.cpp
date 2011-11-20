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

    // Zero all PWM update values.
    for (int i=0; i<4; i++) {
        pwmOutUpdate[i] = 0;
    }

    PID[PID_ROT_X].P = 3.2;
    PID[PID_ROT_X].I = 0.0;
    PID[PID_ROT_X].D = -0.060;

    PID[PID_ROT_Y].P = 3.2;
    PID[PID_ROT_Y].I = 0.0;
    PID[PID_ROT_Y].D = -0.060;

    PID[PID_ROT_Z].P = 50.0;
    PID[PID_ROT_Z].I = 0.0;
    PID[PID_ROT_Z].D = 1.0;

    //PID[PID_MOTOR_T].P = 0.01;
    //PID[PID_MOTOR_T].I = 0;
    //PID[PID_MOTOR_T].D = 0;

    //PID[PID_MOTOR_R].P = 0.01;
    //PID[PID_MOTOR_R].I = 0;
    //PID[PID_MOTOR_R].D = 0;

    //PID[PID_MOTOR_L].P = 0.01;
    //PID[PID_MOTOR_L].I = 0;
    //PID[PID_MOTOR_L].D = 0;

    //PID[PID_SERVO_T].P = 0.1;
    //PID[PID_SERVO_T].I = 0;
    //PID[PID_SERVO_T].D = 0;

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

    // Update axisVal only if okayToFly is true.
    if (okayToFly) {
        // ====================================================================
        // Shift serial input values [1, 251] to correct range for each axis. Z
        // stays positive for ease of calculation.
        // ====================================================================
        axisVal[SX] = (float) serInput[SX] - 126;   // [-125, 125]
        axisVal[SY] = (float) serInput[SY] - 126;   // [-125, 125]
        axisVal[ST] = (float) serInput[ST] - 126;   // [-125, 125]
        axisVal[SZ] = (float) serInput[SZ] - 1;     // [0, 250]

        // ====================================================================
        // Calculate target rotation vector based on joystick input scaled to a
        // maximum rotation of PI/6.
        //
        // TODO: The first two are approximations! Need to figure out how to
        // properly use the DCM.
        // ====================================================================
        pidRot[0] = updatePID(-axisVal[SY]/125 * PI/6,
                              gyroDCM[1][2],
                              PID[PID_ROT_X]);
        pidRot[1] = updatePID(axisVal[SX]/125 * PI/6,
                              -gyroDCM[0][2],
                              PID[PID_ROT_Y]);
        pidRot[2] = updatePID(-axisVal[ST]/125 * PI/6,
                              0,//atan2(gyroDCM[0][1], gyroDCM[0][0]),
                              PID[PID_ROT_Z]);

        //targetRot[0] = -(axisVal[SY]/125 * PI/6 + gyroDCM[1][2]);
        //targetRot[1] =  (axisVal[SX]/125 * PI/6 + gyroDCM[0][2]);
        //targetRot[2] = -(axisVal[ST]/125 * PI/6);

        // ====================================================================
        // Calculate motor/servo values.
        //     MOTOR_X_OFFSET: Offset starting motor values to account for
        //                     chassis imbalance.
        //     MOTOR_X_SCALE: Scale targetRot.
        //     axisVal[SZ]: Throttle.
        //
        // TODO: MOTOR_X_SCALE should be replaced by actual PID gains. Besides,
        // it's incorrect to scale in the negative direction if the
        // corresponding arm is heavier.
        // TODO: The last term for each pwmOut is INACCURATE. Fix this.
        // ====================================================================

        // MOTORVAL SCHEME 1
        //pwmOut[MOTOR_T] = MOTOR_T_OFFSET + axisVal[SZ] + MOTOR_T_SCALE * (-targetRot[0]/2);
        //pwmOut[MOTOR_R] = MOTOR_R_OFFSET + axisVal[SZ] + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]/sqrt(3));
        //pwmOut[MOTOR_L] = MOTOR_L_OFFSET + axisVal[SZ] + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]/sqrt(3));

        // MOTORVAL SCHEME 2
        //pwmOut[MOTOR_T] = MOTOR_T_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_T_SCALE * (-targetRot[0]);
        //pwmOut[MOTOR_R] = MOTOR_R_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]);
        //pwmOut[MOTOR_L] = MOTOR_L_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]);

        // MOTORVAL SCHEME 3 (targetRot to motorVal conversion)
        //pwmOut[MOTOR_T] = MOTOR_T_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_T_SCALE * (-targetRot[0]*2);
        //pwmOut[MOTOR_R] = MOTOR_R_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_R_SCALE * ( targetRot[0] - targetRot[1]*sqrt(3));
        //pwmOut[MOTOR_L] = MOTOR_L_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + MOTOR_L_SCALE * ( targetRot[0] + targetRot[1]*sqrt(3));
        //pwmOut[SERVO_T] = TAIL_SERVO_DEFAULT_POSITION + TAIL_SERVO_SCALE * targetRot[2];

        // MOTORVAL SCHEME 4 (WITH PID)
        // Generate PID update values to output to PWM.
        //pwmOutUpdate[MOTOR_T] = updatePID(-targetRot[0]*2, 0, PID[PID_MOTOR_T]);
        //pwmOutUpdate[MOTOR_R] = updatePID(targetRot[0] - targetRot[1]*sqrt(3), 0, PID[PID_MOTOR_R]);
        //pwmOutUpdate[MOTOR_L] = updatePID(targetRot[0] + targetRot[1]*sqrt(3), 0, PID[PID_MOTOR_L]);
        //pwmOutUpdate[SERVO_T] = updatePID(targetRot[2], 0, PID[PID_SERVO_T]);

        // Add pwmOutUpdate to pwmOut along with motor offsets and throttle.
        //pwmOut[MOTOR_T] = updatePID(MOTOR_T_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) - (targetRot[0]*2),
        //                            pwmOut[MOTOR_T],
        //                            PID[PID_MOTOR_T]);
        //pwmOut[MOTOR_R] = updatePID(MOTOR_R_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + (targetRot[0] - targetRot[1]*sqrt(3)),
        //                            pwmOut[MOTOR_R],
        //                            PID[PID_MOTOR_R]);
        //pwmOut[MOTOR_L] = updatePID(MOTOR_L_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + (targetRot[0] + targetRot[1]*sqrt(3)),
        //                            pwmOut[MOTOR_L],
        //                            PID[PID_MOTOR_L]);
        //pwmOut[SERVO_T] = updatePID(TAIL_SERVO_DEFAULT_POSITION + targetRot[2],
        //                            pwmOut[SERVO_T],
        //                            PID[PID_SERVO_T]);

        // MOTORVAL SCHEME 5 (pidRot to motorVal conversion)
        pwmOut[MOTOR_T] = MOTOR_T_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) + -pidRot[0]*2;
        pwmOut[MOTOR_R] = MOTOR_R_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) +  pidRot[0] - pidRot[1]*sqrt(3);
        pwmOut[MOTOR_L] = MOTOR_L_OFFSET + (TMIN + axisVal[SZ]*(TMAX-TMIN)/250) +  pidRot[0] + pidRot[1]*sqrt(3);
        pwmOut[SERVO_T] = TAIL_SERVO_DEFAULT_POSITION + pidRot[2];

        // DEPRECATED.
        //pwmOut[MOTOR_T] = MOTOR_T_SCALE * (MOTOR_T_OFFSET + axisVal[SZ] + 0.6667*(GYRO_COEFF*gVal[0] + commandPitch));   // Watch out for floats vs. ints
        //pwmOut[MOTOR_R] = MOTOR_R_SCALE * (MOTOR_R_OFFSET + axisVal[SZ] + 0.3333*(-GYRO_COEFF*gVal[0] - commandPitch) + (GYRO_COEFF*gVal[1] - commandRoll)/sqrt(3));
        //pwmOut[MOTOR_L] = MOTOR_L_SCALE * (MOTOR_L_OFFSET + axisVal[SZ] + 0.3333*(-GYRO_COEFF*gVal[0] - commandPitch) + (-GYRO_COEFF*gVal[1] + commandRoll)/sqrt(3));


        // ====================================================================
        // After finding the maximum and minimum motor values, limit, but NOT
        // fit, motor values to minimum and maximum throttle [TMIN, TMAX]).
        // Doing this incorrectly will result in motor values seemingly stuck
        // mostly at either extremes.
        // ====================================================================
        mapUpper = pwmOut[MOTOR_T] > pwmOut[MOTOR_R] ? pwmOut[MOTOR_T] : pwmOut[MOTOR_R];
        mapUpper = mapUpper > pwmOut[MOTOR_L] ? mapUpper : pwmOut[MOTOR_L];
        mapUpper = mapUpper > TMAX ? mapUpper : TMAX;

        mapLower = pwmOut[MOTOR_T] < pwmOut[MOTOR_R] ? pwmOut[MOTOR_T] : pwmOut[MOTOR_R];
        mapLower = mapLower < pwmOut[MOTOR_L] ? mapLower : pwmOut[MOTOR_L];
        mapLower = mapLower < TMIN ? mapLower : TMIN;

        // We shouldn't have to use these, but uncomment the following two
        // lines if pwmOut goes crazy and makes mapUpper lower than mapLower:
        //mapUpper = mapUpper > TMIN ? mapUpper : TMIN+1;
        //mapLower = mapLower < TMAX ? mapLower : TMAX-1;

        // ====================================================================
        // If map bounds are reasonable, remap range to [mapLower, mapUpper].
        // Otherwise, kill motors. Note that map(), an Arduino function, does
        // integer math and truncates fractions.
        //
        // TODO: pwmOut (and other quantities the Pilot calculates) should be
        // an integer representing the number of milliseconds of PWM duty
        // cycle.
        // ====================================================================
        for (int i=0; i<3; i++) {
            if (mapUpper > mapLower) {
                pwmOut[i] = map(pwmOut[i], mapLower, mapUpper, TMIN, TMAX);
            }
            else {
                pwmOut[i] = TMIN;
            }
        }

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
    // ========================================================================
    // When communication is lost, pilot should set a bunch of stuff to safe
    // values.
    // ========================================================================
    serInput[SX] = 126;
    serInput[SY] = 126;
    serInput[ST] = 126;
    serInput[SZ] = 3;
    okayToFly = false;
    pwmOut[MOTOR_T] = TMIN;
    pwmOut[MOTOR_R] = TMIN;
    pwmOut[MOTOR_L] = TMIN;
    pwmOut[SERVO_T] = 90;
    
    #ifdef DEBUG
    spln("Pilot ejected!");
    #endif
}

