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

    serRead = 0;

    // Assume serial inputs, all axes zeroed.
    serInput[SX] = 125;
    serInput[SY] = 125;
    serInput[SZ] = 125;
    /* Keep Z value at some non-zero value (albeit very low so the tricopter 
     * doesn't fly off if something goes awry) so user is forced to adjust 
     * throttle before motors arm. */
    serInput[ST0] = 3;
    serInput[ST1] = 3;
    serInput[SH0] = 125;
    serInput[SH1] = 125;

    // Zero rotation values.
    for (int i=0; i<3; i++) {
        targetAngPos[i] = 0;
        currentAngPos[i] = 0;
        pidAngPos[i] = 0;
        pidAngVel[i] = 0;
        currentAngPos[i] = 0;
    }
    ang_pos_xy_cap = 0;
    ang_vel_xy_cap = 0;
    throttle = 0;
    throttleEnabled = 0;

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

        okayToFly = false;
    }


    // ANGULAR POSITION CONTROL FLIGHT MODE
    if (flightMode == HOVER) {
        // ====================================================================
        // Calculate target rotation vector based on joystick input scaled to a
        // maximum rotation of PI/6.
        //
        // TODO: The first two are approximations! Need to figure out how to
        // properly use the DCM.
        // ====================================================================
        targetAngPos[0] = -joy.axes[SY] * ang_pos_xy_cap;
        targetAngPos[1] =  joy.axes[SX] * ang_pos_xy_cap;
        targetAngPos[2] += joy.axes[SZ] * ANG_VEL_Z_CAP * CONTROL_LOOP_INTERVAL * MASTER_DT / 1000000;

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
        currentAngPos[0] = -atan2(bodyDCM[2][1], bodyDCM[2][2]) * bodyDCM[0][0] + atan2(bodyDCM[2][0], bodyDCM[2][2]) * bodyDCM[0][1];
        currentAngPos[1] =  atan2(bodyDCM[2][0], bodyDCM[2][2]) * bodyDCM[1][1] - atan2(bodyDCM[2][1], bodyDCM[2][2]) * bodyDCM[1][0];

        if (ABS(currentAngPos[0]) < PI/4 && ABS(currentAngPos[1]) < PI/4) {
            currentAngPos[2] = atan2(bodyDCM[0][1], bodyDCM[0][0]);
        }
        else {
            currentAngPos[2] = targetAngPos[2];
        }

        // Keep abs(targetAngPos[i] - currentAngPos[i]) within [-PI, PI].
        // This way, nothing bad happens as we rotate to any angle in [-PI,
        // PI].   TODO: This would be more effective if the current angular
        // velocity is taken into consideration before calculating
        // currentAngPos.
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
        targetAngVel[0] = -joy.axes[SY] * ang_vel_xy_cap;
        targetAngVel[1] =  joy.axes[SX] * ang_vel_xy_cap;
        targetAngVel[2] =  joy.axes[SZ] * ANG_VEL_Z_CAP;
    }


    angular_velocity_controller(targetAngVel, gVec, pwmShift);

    // TODO: throttleEnabled should be implemented in ground station comm.
    throttle = throttleEnabled * (0.8*joy.axes[ST1] + 0.2*joy.axes[ST0]) * (TMAX-TMIN);

    // If in hover mode, increase throttle based on chassis tilt, but not past
    // around 37 degrees.
    if (flightMode == HOVER) {
        throttle = throttle / MAX(bodyDCM[2][2], 0.8);
    }

    calculate_pwm_outputs(throttle, pwmShift, pwmOut);
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
    joy.axes[SX]  = ((float) serInput[SX] - 125)/125;    // [-1.0, 1.0]
    joy.axes[SY]  = ((float) serInput[SY] - 125)/125;    // [-1.0, 1.0]
    joy.axes[SZ]  = ((float) serInput[SZ] - 125)/125;    // [-1.0, 1.0]
    joy.axes[ST0] = ((float) serInput[ST0])/250;         // [ 0.0, 1.0]
    joy.axes[ST1] = ((float) serInput[ST1])/250;         // [ 0.0, 1.0]
    joy.axes[SH0] = ((float) serInput[SH0] - 125)/125;   // [-1.0, 1.0]
    joy.axes[SH1] = ((float) serInput[SH1] - 125)/125;   // [-1.0, 1.0]

    // ========================================================================
    // Set button values. We utilize only the lower 7 bits, since doing
    // otherwise would cause overlaps with serial headers.
    // ========================================================================
    for (int i=0; i<7; i++) {
        joy.buttons[i]   = serInput[SB0] & (1<<i);
        joy.buttons[7+i] = serInput[SB1] & (1<<i);
    }

    trimAngle[0] -= joy.axes[SH1] * COMM_LOOP_INTERVAL / MASTER_DT;
    trimAngle[1] += joy.axes[SH0] * COMM_LOOP_INTERVAL / MASTER_DT;
}

void Pilot::process_joystick_buttons(void) {
    // Disable throttle.
    if (joy.buttons[BUTTON_ZERO_THROTTLE]) {
        throttleEnabled = 0;
    }
    else {
        throttleEnabled = 1;
    }

    // Determine flight mode.
    if (joy.buttons[BUTTON_SOFT_ACRO_MODE] ||
            joy.buttons[BUTTON_HARD_ACRO_MODE]) {
        flightMode = ACRO;
    }
    else {
        flightMode = HOVER;
    }

    // Soft acro mode (using low angular velocity cap).
    if (joy.buttons[BUTTON_SOFT_ACRO_MODE]) {
        ang_vel_xy_cap = ANG_VEL_XY_CAP_LOW;
    }
    // Hard acro mode (using high angular velocity cap) or normal operation.
    else {
        ang_vel_xy_cap = ANG_VEL_XY_CAP_HIGH;
    }

    // Increase the angular position cap.
    if (joy.buttons[BUTTON_INC_ANG_POS_CAP]) {
        ang_pos_xy_cap = ANG_POS_XY_CAP_HIGH;
    }
    else {
        ang_pos_xy_cap = ANG_POS_XY_CAP_LOW;
    }

    // "Reset" targetAngPos[2] to currentAngPos[2] if thumb button is pressed.
    if (joy.buttons[BUTTON_RESET_YAW]) {
        targetAngPos[2] = currentAngPos[2];
        targetAngPos[2] += joy.axes[SZ] * ANG_VEL_Z_CAP;
    }

    #ifdef SEND_PID_DATA
    // Adjust gains on-the-fly.
    if (joy.buttons[BUTTON_INCREMENT_GAIN]) {
        if (joy.buttons[BUTTON_ANG_POS_XY_P_GAIN]) {
            PID[PID_ANG_POS_X].P = MAX(PID[PID_ANG_POS_X].P + 0.1, 0);
            PID[PID_ANG_POS_Y].P = MAX(PID[PID_ANG_POS_Y].P + 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_XY_P_GAIN]) {
            PID[PID_ANG_VEL_X].P = MAX(PID[PID_ANG_VEL_X].P + 0.1, 0);
            PID[PID_ANG_VEL_Y].P = MAX(PID[PID_ANG_VEL_Y].P + 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_XY_D_GAIN]) {
            PID[PID_ANG_VEL_X].D = MIN(PID[PID_ANG_VEL_X].D - 0.01, 0);
            PID[PID_ANG_VEL_Y].D = MIN(PID[PID_ANG_VEL_Y].D - 0.01, 0);
        }
        if (joy.buttons[BUTTON_ANG_POS_Z_P_GAIN]) {
            PID[PID_ANG_POS_Z].P = MAX(PID[PID_ANG_POS_Z].P + 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_Z_P_GAIN]) {
            PID[PID_ANG_VEL_Z].P = MAX(PID[PID_ANG_VEL_Z].P + 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_Z_D_GAIN]) {
            PID[PID_ANG_VEL_Z].D = MIN(PID[PID_ANG_VEL_Z].D - 0.01, 0);
        }
    }
    else if (joy.buttons[BUTTON_DECREMENT_GAIN]) {
        if (joy.buttons[BUTTON_ANG_POS_XY_P_GAIN]) {
            PID[PID_ANG_POS_X].P = MAX(PID[PID_ANG_POS_X].P - 0.1, 0);
            PID[PID_ANG_POS_Y].P = MAX(PID[PID_ANG_POS_Y].P - 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_XY_P_GAIN]) {
            PID[PID_ANG_VEL_X].P = MAX(PID[PID_ANG_VEL_X].P - 0.1, 0);
            PID[PID_ANG_VEL_Y].P = MAX(PID[PID_ANG_VEL_Y].P - 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_XY_D_GAIN]) {
            PID[PID_ANG_VEL_X].D = MIN(PID[PID_ANG_VEL_X].D + 0.01, 0);
            PID[PID_ANG_VEL_Y].D = MIN(PID[PID_ANG_VEL_Y].D + 0.01, 0);
        }
        if (joy.buttons[BUTTON_ANG_POS_Z_P_GAIN]) {
            PID[PID_ANG_POS_Z].P = MAX(PID[PID_ANG_POS_Z].P - 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_Z_P_GAIN]) {
            PID[PID_ANG_VEL_Z].P = MAX(PID[PID_ANG_VEL_Z].P - 0.1, 0);
        }
        if (joy.buttons[BUTTON_ANG_VEL_Z_D_GAIN]) {
            PID[PID_ANG_VEL_Z].D = MIN(PID[PID_ANG_VEL_Z].D + 0.01, 0);
        }
    }
    #endif // SEND_PID_DATA
}

