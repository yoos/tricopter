/*! \file motors.h
 *  \author Soo-Hyun Yoo
 *  \brief Calculate PWM outputs for motors and servo based on target rotation or translation input.
 *
 *  Details.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "globals.h"
#include "pid.h"

// Calculate throttle shifts for the individual motors based on desired rate
// inputs.
void angular_rate_controller (float* desired_rate, float* current_rate, int16_t* pwmShift) {
    pidAngRate[0] = updatePID(desired_rate[0], current_rate[0], PID[PID_ANG_RATE_X]);
    pidAngRate[1] = updatePID(desired_rate[1], current_rate[1], PID[PID_ANG_RATE_Y]);
    pidAngRate[2] = updatePID(desired_rate[2], current_rate[2], PID[PID_ANG_RATE_Z]);

    for (int i=0; i<3; i++) {
        pwmShift[i] = (int16_t) pidAngRate[i];
    }
}

void angular_position_controller (float* desired_pos, float* current_pos, float* desired_rate) {
    pidAngPos[0] = updatePID(desired_pos[0], current_pos[0], PID[PID_ANG_POS_X]);
    pidAngPos[1] = updatePID(desired_pos[1], current_pos[1], PID[PID_ANG_POS_Y]);
    pidAngPos[2] = updatePID(desired_pos[2], current_pos[2], PID[PID_ANG_POS_Z]);

    for (int i=0; i<3; i++) {
        desired_rate[i] = pidAngPos[i];
    }
}

void calculate_pwm_outputs(float pwmThrottle, int16_t* pwmShift, int16_t* pwmOut) {
    // ====================================================================
    // Calculate motor/servo values.
    //     MOTOR_X_OFFSET: Offset starting motor values to account for
    //                     chassis imbalance.
    //     MOTOR_X_SCALE: Scale targetRot.
    //     joy.axes[SZ]: Throttle.
    //
    // TODO: MOTOR_X_SCALE should be replaced by actual PID gains. Besides,
    // it's incorrect to scale in the negative direction if the
    // corresponding arm is heavier.
    // TODO: The last term for each pwmOut is INACCURATE. Fix this.
    // ====================================================================
    pwmOut[SERVO_T] = SERVO_US_NEUTRAL + pwmShift[2];

    pwmOut[MOTOR_T] = (pwmThrottle + -pwmShift[0]) / cos(((float) pwmOut[SERVO_T]-SERVO_US_ZERO)/SERVO_US_PER_RAD);
    pwmOut[MOTOR_R] =  pwmThrottle +  pwmShift[0] - pwmShift[1]*sqrt(3);
    pwmOut[MOTOR_L] =  pwmThrottle +  pwmShift[0] + pwmShift[1]*sqrt(3);

    pwmOut[MOTOR_T] = TMIN + MOTOR_T_OFFSET + MOTOR_T_SCALE * pwmOut[MOTOR_T];
    pwmOut[MOTOR_R] = TMIN + MOTOR_R_OFFSET + MOTOR_R_SCALE * pwmOut[MOTOR_R];
    pwmOut[MOTOR_L] = TMIN + MOTOR_L_OFFSET + MOTOR_L_SCALE * pwmOut[MOTOR_L];

    // ====================================================================
    // After finding the maximum and minimum motor values, limit, but NOT
    // fit, motor values to minimum and maximum throttle [TMIN, TMAX]).
    // Doing this incorrectly will result in motor values seemingly stuck
    // mostly at either extremes.
    // ====================================================================
    int mapUpper = pwmOut[MOTOR_T] > pwmOut[MOTOR_R] ? pwmOut[MOTOR_T] : pwmOut[MOTOR_R];
    mapUpper = mapUpper > pwmOut[MOTOR_L] ? mapUpper : pwmOut[MOTOR_L];
    mapUpper = mapUpper > TMAX ? mapUpper : TMAX;

    int mapLower = pwmOut[MOTOR_T] < pwmOut[MOTOR_R] ? pwmOut[MOTOR_T] : pwmOut[MOTOR_R];
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
    // ====================================================================
    for (int i=0; i<3; i++) {
        if (mapUpper > mapLower) {
            pwmOut[i] = map(pwmOut[i], mapLower, mapUpper, TMIN, TMAX);
        }
        else {
            pwmOut[i] = TMIN;
        }
    }
}

#endif // MOTORS_H

