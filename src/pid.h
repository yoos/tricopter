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

    return PIDparameters.P * proportional +
           PIDparameters.I * PIDparameters.integral +
           PIDparameters.D * derivative;
}

// void zeroIntegralproportional() __attribute__ ((noinline));
// void zeroIntegralproportional() {
//   for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
//     PID[axis].integral = 0;
//     PID[axis].previousPIDTime = currentTime;
//   }
// }

#endif // PID_H

