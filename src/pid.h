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

    //// Process X and Y rotation vectors differently.
    //if (PIDparameters.id == PID_ANG_POS_X ||
    //    PIDparameters.id == PID_ANG_POS_Y) {
    //    // Cap maximum error.
    //    if (proportional > TARGET_ANGLE_CAP) {
    //        proportional = TARGET_ANGLE_CAP;
    //    }
    //    else if (proportional < -TARGET_ANGLE_CAP) {
    //        proportional = -TARGET_ANGLE_CAP;
    //    }
    //}

    //// For rate controller, get rate directly from gyro outputs to minimize
    //// noise.
    //else if (PIDparameters.id == PID_ANG_RATE_X) {
    //    derivative = gVec[0];
    //}
    //else if (PIDparameters.id == PID_ANG_RATE_Y) {
    //    derivative = gVec[1];
    //}

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

