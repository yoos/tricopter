#ifndef PID_H
#define PID_H

#include "globals.h"

float updatePID(float targetValue, float currentValue, struct PIDdata &PIDparameters) {
    float proportional;
    float derivative;
    float deltaPIDTime = (float) MASTER_DT * CONTROL_LOOP_INTERVAL / 1000;

    proportional = targetValue - currentValue;

    PIDparameters.integral += proportional * deltaPIDTime;

    derivative = (currentValue - PIDparameters.lastValue) / deltaPIDTime;
    PIDparameters.lastValue = currentValue;

    // Zero integral once overshoot is detected.
    if ((proportional < 0 && derivative > 0) ||
        (proportional > 0 && derivative < 0)) {
        PIDparameters.integral = 0;
    }

    // NOTE: The P, I, and D gains should end up being no more than a single
    // degree of magnitude away from 1.
    return PIDparameters.P * proportional +
           PIDparameters.I * PIDparameters.integral +
           PIDparameters.D * derivative;
}

#endif // PID_H

