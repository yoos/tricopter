/*! \file pilot.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for Pilot class.
 *
 *  Details.
 */

#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    okayToFly = false;

    // Assume serial inputs, all axes zeroed.
    serInput[SX] = 125;
    serInput[SY] = 125;
    serInput[ST] = 125;
    /* Keep Z value at some non-zero value (albeit very low so the tricopter 
     * doesn't fly off if something goes awry) so user is forced to adjust 
     * throttle before motors arm. */
    serInput[SZ] = 3;

    // Zero rotation values.
    for (int i=0; i<3; i++) {
        targetAngPos[i] = 0;
        currentAngPos[i] = 0;
        pidAngPos[i] = 0;
        pidAngVel[i] = 0;
        currentAngPos[i] = 0;
    }

    // Initialize trim to 0.
    throttleTrim = 0;

    // Set PID data ID so the PID function can apply appropriate caps, etc.
    PID[PID_ANG_POS_X].id = PID_ANG_POS_X;
    PID[PID_ANG_POS_Y].id = PID_ANG_POS_Y;
    PID[PID_ANG_POS_Z].id = PID_ANG_POS_Z;
    PID[PID_ANG_VEL_X].id = PID_ANG_VEL_X;
    PID[PID_ANG_VEL_Y].id = PID_ANG_VEL_Y;
    PID[PID_ANG_VEL_Z].id = PID_ANG_VEL_Z;

    // Set dt.
    float deltaPIDTime = (float) MASTER_DT * CONTROL_LOOP_INTERVAL / 1000000;   // Time difference in seconds.
    PID[PID_ANG_POS_X].deltaPIDTime = deltaPIDTime;
    PID[PID_ANG_POS_Y].deltaPIDTime = deltaPIDTime;
    PID[PID_ANG_POS_Z].deltaPIDTime = deltaPIDTime;
    PID[PID_ANG_VEL_X].deltaPIDTime = deltaPIDTime;
    PID[PID_ANG_VEL_Y].deltaPIDTime = deltaPIDTime;
    PID[PID_ANG_VEL_Z].deltaPIDTime = deltaPIDTime;

    // Set initial PID gains.
    PID[PID_ANG_POS_X].P = PID[PID_ANG_POS_Y].P = XY_ANG_POS_P_GAIN;
    PID[PID_ANG_POS_X].I = PID[PID_ANG_POS_Y].I = XY_ANG_POS_I_GAIN;
    PID[PID_ANG_POS_X].D = PID[PID_ANG_POS_Y].D = XY_ANG_POS_D_GAIN;

    PID[PID_ANG_POS_Z].P = Z_ANG_POS_P_GAIN;
    PID[PID_ANG_POS_Z].I = Z_ANG_POS_I_GAIN;
    PID[PID_ANG_POS_Z].D = Z_ANG_POS_D_GAIN;

    PID[PID_ANG_VEL_X].P = PID[PID_ANG_VEL_Y].P = XY_ANG_VEL_P_GAIN;
    PID[PID_ANG_VEL_X].I = PID[PID_ANG_VEL_Y].I = XY_ANG_VEL_I_GAIN;
    PID[PID_ANG_VEL_X].D = PID[PID_ANG_VEL_Y].D = XY_ANG_VEL_D_GAIN;

    PID[PID_ANG_VEL_Z].P = Z_ANG_VEL_P_GAIN;
    PID[PID_ANG_VEL_Z].I = Z_ANG_VEL_I_GAIN;
    PID[PID_ANG_VEL_Z].D = Z_ANG_VEL_D_GAIN;

    flightMode = OFF;

    numGoodComm = 0;   // Number of good communication packets.
    numBadComm = 0;   // Number of bad communication packets.
}

void Pilot::listen() {
    int serCount = Serial.available();
    for (int h=0; h<MIN(serCount, SER_READ_CHUNK_LEN); h++) {
        rBuf[rIndex] = Serial.read();

        if (rBuf[rIndex] == SERHEAD) {   // Receive header.
            hasFood = true;   // Prepare food for watchdog.
            for (int i=0; i<SER_PACKET_LEN; i++) {
                uint8_t serVal = rBuf[(SER_READ_BUF_LEN + rIndex - SER_PACKET_LEN + i) % SER_READ_BUF_LEN];
                if (serVal >= INPUT_MIN && serVal <= INPUT_MAX) {
                    serInput[i] = serVal;
                    okayToFly = true;
                }
                else {
                    okayToFly = false;
                    i = SER_PACKET_LEN;   // Don't run this loop anymore.
                    // Flush remaining buffer to avoid taking in the wrong values.
                }
            }

            if (okayToFly) {
                numGoodComm++;
            }
            else {
                numBadComm++;
            }
        }

        rIndex = (rIndex+1) % SER_READ_BUF_LEN;
    }
    //sp("(");
    //sp(numGoodComm);
    //sp("/");
    //sp(numBadComm);
    //sp(" ");
    //sp((int) rIndex);
    //sp(" ");
    //sp((int) serCount);
    //spln(") ");
}

void Pilot::fly() {
    if (okayToFly) {
        //spln("");
        //sp("fly ");
        //spln(micros());

        update_joystick_input();
        process_joystick_buttons();

        // ANGULAR POSITION CONTROL FLIGHT MODE
        if (flightMode == HOVER) {
            // ====================================================================
            // Calculate target rotation vector based on joystick input scaled to a
            // maximum rotation of PI/6.
            //
            // TODO: The first two are approximations! Need to figure out how to
            // properly use the DCM.
            // ====================================================================
            targetAngPos[0] = -joy.axes[SY]/125 * TARGET_ANG_POS_CAP;
            targetAngPos[1] =  joy.axes[SX]/125 * TARGET_ANG_POS_CAP;
            targetAngPos[2] += joy.axes[ST]/125 * Z_ROT_SPEED / (MASTER_DT * CONTROL_LOOP_INTERVAL);

            // Keep targetAngPos within [-PI, PI].
            for (int i=0; i<3; i++) {
                if (targetAngPos[i] > PI) {
                    targetAngPos[i] -= 2*PI;
                }
                else if (targetAngPos[i] < -PI) {
                    targetAngPos[i] += 2*PI;
                }
            }

            // ====================================================================
            // Calculate current rotation vector (Euler angles) from DCM and make
            // appropriate modifications to make PID calculations work later.
            // ====================================================================
            currentAngPos[0] = bodyDCM[1][2];
            currentAngPos[1] = -bodyDCM[0][2];
            currentAngPos[2] = atan2(bodyDCM[0][1], bodyDCM[0][0]);

            // Keep abs(targetAngPos[i] - currentAngPos[i]) within [-PI, PI]. This way,
            // nothing bad happens as we rotate to any angle in [-PI, PI].
            for (int i=0; i<3; i++) {
                if (targetAngPos[i] - currentAngPos[i] > PI) {
                    currentAngPos[i] += 2*PI;
                }
                else if (targetAngPos[i] - currentAngPos[i] < -PI) {
                    currentAngPos[i] -= 2*PI;
                }
            }

            angular_position_controller(targetAngPos, currentAngPos, targetAngVel);
        }


        // ANGULAR VELOCITY CONTROL FLIGHT MODE
        else if (flightMode == ACRO) {
            targetAngVel[0] = -joy.axes[SY]/125 * TARGET_ANG_VEL_CAP;
            targetAngVel[1] =  joy.axes[SX]/125 * TARGET_ANG_VEL_CAP;
            targetAngVel[2] =  joy.axes[ST]/125 * Z_ROT_SPEED;
        }


        angular_velocity_controller(targetAngVel, gVec, pwmShift);

        throttle = throttleTrim + joy.axes[SZ] * (TMAX-TMIN) / 250;

        calculate_pwm_outputs(throttle, pwmShift, pwmOut);

        okayToFly = false;
    }
}

void Pilot::die() {
    // ========================================================================
    // When communication is lost, pilot should set a bunch of stuff to safe
    // values.
    // ========================================================================
    okayToFly = false;
    pwmOut[MOTOR_T] = 0;
    pwmOut[MOTOR_R] = 0;
    pwmOut[MOTOR_L] = 0;
    pwmOut[SERVO_T] = 1900;
}

void Pilot::update_joystick_input(void) {
    // ========================================================================
    // Shift serial input values [0, 250] to correct range for each axis. Z
    // stays positive for ease of calculation.
    // ========================================================================
    joy.axes[SX] = (float) serInput[SX] - 125;   // [-125, 125]
    joy.axes[SY] = (float) serInput[SY] - 125;   // [-125, 125]
    joy.axes[ST] = (float) serInput[ST] - 125;   // [-125, 125]
    joy.axes[SZ] = (float) serInput[SZ];         // [   0, 250]

    // ========================================================================
    // Set button values. We utilize only the lower 7 bits, since doing
    // otherwise would cause overlaps with serial headers.
    // ========================================================================
    for (int i=0; i<7; i++) {
        joy.buttons[i]   = serInput[SB1] & (1<<i);
        joy.buttons[7+i] = serInput[SB2] & (1<<i);
    }
}

void Pilot::process_joystick_buttons(void) {
    // "Reset" targetAngPos[2] to currentAngPos[2] if thumb button is pressed.
    if (joy.buttons[BUTTON_RESET_YAW]) {
        targetAngPos[2] = currentAngPos[2];
        targetAngPos[2] += joy.axes[ST]/125 * Z_ROT_SPEED;
    }

    // Zero integral.
    if (joy.buttons[BUTTON_ZERO_INTEGRAL]) {
        PID[PID_ANG_POS_X].integral = 0;
        PID[PID_ANG_POS_Y].integral = 0;
    }

    // Enable acro mode (velocity control).
    if (joy.buttons[BUTTON_ACRO_MODE]) {
        flightMode = ACRO;
    }
    else {
        flightMode = HOVER;
    }

    // Trim throttle value.
    if (joy.buttons[BUTTON_DECREASE_TRIM] && joy.buttons[BUTTON_INCREASE_TRIM]) {
        throttleTrim = 0;
    }
    else if (joy.buttons[BUTTON_DECREASE_TRIM]) {
        throttleTrim--;
    }
    else if (joy.buttons[BUTTON_INCREASE_TRIM]) {
        throttleTrim++;
    }

    // Adjust gains on-the-fly.
    if (joy.buttons[BUTTON_DECREASE_XY_ANG_POS_P_GAIN] && joy.buttons[BUTTON_INCREASE_XY_ANG_POS_P_GAIN]) {
        PID[PID_ANG_POS_X].P = XY_ANG_POS_P_GAIN;
        PID[PID_ANG_POS_Y].P = XY_ANG_POS_P_GAIN;
    }
    else if (joy.buttons[BUTTON_DECREASE_XY_ANG_POS_P_GAIN] && PID[PID_ANG_POS_X].P > 0) {
        PID[PID_ANG_POS_X].P -= 1.0;
        PID[PID_ANG_POS_Y].P -= 1.0;
    }
    else if (joy.buttons[BUTTON_INCREASE_XY_ANG_POS_P_GAIN]) {
        PID[PID_ANG_POS_X].P += 1.0;
        PID[PID_ANG_POS_Y].P += 1.0;
    }

    if (joy.buttons[BUTTON_DECREASE_XY_ANG_VEL_P_GAIN] && joy.buttons[BUTTON_INCREASE_XY_ANG_VEL_P_GAIN]) {
        PID[PID_ANG_VEL_X].P = XY_ANG_VEL_P_GAIN;
        PID[PID_ANG_VEL_Y].P = XY_ANG_VEL_P_GAIN;
    }
    else if (joy.buttons[BUTTON_DECREASE_XY_ANG_VEL_P_GAIN] && PID[PID_ANG_VEL_X].P > 0) {
        PID[PID_ANG_VEL_X].P -= 1.0;
        PID[PID_ANG_VEL_Y].P -= 1.0;
    }
    else if (joy.buttons[BUTTON_INCREASE_XY_ANG_VEL_P_GAIN]) {
        PID[PID_ANG_VEL_X].P += 1.0;
        PID[PID_ANG_VEL_Y].P += 1.0;
    }

    if (joy.buttons[BUTTON_DECREASE_XY_ANG_VEL_D_GAIN] && joy.buttons[BUTTON_INCREASE_XY_ANG_VEL_D_GAIN]) {
        PID[PID_ANG_VEL_X].D = XY_ANG_VEL_D_GAIN;
        PID[PID_ANG_VEL_Y].D = XY_ANG_VEL_D_GAIN;
    }
    else if (joy.buttons[BUTTON_DECREASE_XY_ANG_VEL_D_GAIN] && PID[PID_ANG_VEL_X].D < 0) {
        PID[PID_ANG_VEL_X].D += 0.01;
        PID[PID_ANG_VEL_Y].D += 0.01;
    }
    else if (joy.buttons[BUTTON_INCREASE_XY_ANG_VEL_D_GAIN]) {
        PID[PID_ANG_VEL_X].D -= 0.01;
        PID[PID_ANG_VEL_Y].D -= 0.01;
    }
}

