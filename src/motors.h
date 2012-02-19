/*! \file motors.h
 *  \author Soo-Hyun Yoo
 *  \brief Calculate PWM outputs for motors and servo based on target rotation or translation input.
 *
 *  Details.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "globals.h"

void calculate_pwm_output(float inputThrottle, float* inputRot) {
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

    // MOTORVAL SCHEME 5 (pidRot to motorVal conversion)
    //pwmOut[MOTOR_T] = MOTOR_T_OFFSET + (TMIN + joy.axes[SZ]*(TMAX-TMIN)/250) + -pidRot[0]*2;
    //pwmOut[MOTOR_R] = MOTOR_R_OFFSET + (TMIN + joy.axes[SZ]*(TMAX-TMIN)/250) +  pidRot[0] - pidRot[1]*sqrt(3);
    //pwmOut[MOTOR_L] = MOTOR_L_OFFSET + (TMIN + joy.axes[SZ]*(TMAX-TMIN)/250) +  pidRot[0] + pidRot[1]*sqrt(3);
    //pwmOut[SERVO_T] = TAIL_SERVO_DEFAULT_POSITION + pidRot[2];

    // MOTORVAL SCHEME 6
    pwmOut[MOTOR_T] = TMIN + inputThrottle + MOTOR_T_OFFSET + -inputRot[PID_ROT_X];
    pwmOut[MOTOR_R] = TMIN + inputThrottle + MOTOR_R_OFFSET +  inputRot[PID_ROT_X] - inputRot[PID_ROT_Y]*sqrt(3);
    pwmOut[MOTOR_L] = TMIN + inputThrottle + MOTOR_L_OFFSET +  inputRot[PID_ROT_X] + inputRot[PID_ROT_Y]*sqrt(3);
    pwmOut[SERVO_T] = TAIL_SERVO_DEFAULT_POSITION + inputRot[PID_ROT_Z];

    pwmOut[MOTOR_T] = MOTOR_T_SCALE * pwmOut[MOTOR_T];
    pwmOut[MOTOR_R] = MOTOR_R_SCALE * pwmOut[MOTOR_R];
    pwmOut[MOTOR_L] = MOTOR_L_SCALE * pwmOut[MOTOR_L];


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
}

#endif // MOTORS_H

