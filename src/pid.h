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
    float proportional;
    float derivative;
    float deltaPIDTime = (float) MASTER_DT * CONTROL_LOOP_INTERVAL / 1000000;

    proportional = targetValue - currentValue;

    // Cap proportional term if dealing with X or Y rotation vectors.
    if (PIDparameters.id == PID_ROT_X ||
        PIDparameters.id == PID_ROT_Y) {
        if (proportional > ROTATION_CAP) {
            proportional = ROTATION_CAP;
        }
        else if (proportional < -ROTATION_CAP) {
            proportional = -ROTATION_CAP;
        }
    }

    PIDparameters.integral += proportional * deltaPIDTime;
    //PIDparameters.integral *= 0.98;

    derivative = (currentValue - PIDparameters.lastValue) / deltaPIDTime;
    PIDparameters.lastValue = currentValue;

    // Cap derivative term if dealing with X or Y rotation vectors.
    //if (PIDparameters.id == PID_ROT_X ||
    //    PIDparameters.id == PID_ROT_Y) {
    //    if (derivative > ROT_RATE_CAP) {
    //        derivative = ROT_RATE_CAP;
    //    }
    //    else if (derivative < -ROT_RATE_CAP) {
    //        derivative = -ROT_RATE_CAP;
    //    }
    //}

    // Zero integral once overshoot is detected.
    //if ((proportional < 0 && derivative > 0) ||
    //    (proportional > 0 && derivative < 0)) {
    //    //PIDparameters.integral *= 0.95;
    //    PIDparameters.integral = 0.0;
    //}

    // NOTE: The P, I, and D gains should end up being no more than a single
    // degree of magnitude away from each other... maybe.
    return PIDparameters.P * proportional +
           PIDparameters.I * PIDparameters.integral +
           PIDparameters.D * derivative;
}

#endif // PID_H

