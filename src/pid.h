/*! \file pid.h
 *  \author Soo-Hyun Yoo
 *  \brief PID function modified from AeroQuad project.
 *
 *  Details.
 */

#ifndef PID_H
#define PID_H

#include "globals.h"

float updatePID(float targetValue, float currentValue, struct PIDdata &PIDparameters) {
    float proportional = targetValue - currentValue;

    PIDparameters.integral += proportional * PIDparameters.deltaPIDTime;
    //PIDparameters.integral *= 0.98;   // Integral decay

    // Derivative term from difference between last two measured values divided
    // by time interval.
    float derivative = (currentValue - PIDparameters.lastValue) / PIDparameters.deltaPIDTime;   // Per second.
    PIDparameters.lastValue = currentValue;

    // Process X and Y rotation vectors differently.
    if (PIDparameters.id == PID_ROT_X ||
        PIDparameters.id == PID_ROT_Y) {
        // Cap proportional term.
        if (proportional > TARGET_ANGLE_CAP) {
            proportional = TARGET_ANGLE_CAP;
        }
        else if (proportional < -TARGET_ANGLE_CAP) {
            proportional = -TARGET_ANGLE_CAP;
        }

        // Get rate directly from gyro output to minimize noise.
        if (PIDparameters.id == PID_ROT_X) {
            derivative = gVec[0];
        }
        else {
            derivative = gVec[1];
        }

        // Set target rate (rad/s).
        float targetRate = proportional * TARGET_RATE_CAP / TARGET_ANGLE_CAP;
        derivative = derivative - targetRate;

        // Cap change in derivative to help reduce jerkiness.
        if (derivative - PIDparameters.lastDerivative > XY_D_TERM_CAP) {
            derivative = XY_D_TERM_CAP;
        }
        else if (derivative - PIDparameters.lastDerivative < -XY_D_TERM_CAP) {
            derivative = -XY_D_TERM_CAP;
        }
        PIDparameters.lastDerivative = derivative;
    }

    // Zero integral once overshoot is detected.
    //if ((proportional < 0 && derivative > 0) ||
    //    (proportional > 0 && derivative < 0)) {
    //    //PIDparameters.integral *= 0.95;
    //    PIDparameters.integral = 0.0;
    //}

    // NOTE: The P, I, and D gains should end up being no more than a single
    // degree of magnitude away from each other... maybe.
    //
    // TODO: Try returning just the D term so the PID loop is just a rate
    // calculator.
    return PIDparameters.P * proportional +
           PIDparameters.I * PIDparameters.integral +
           PIDparameters.D * derivative;
}

#endif // PID_H

