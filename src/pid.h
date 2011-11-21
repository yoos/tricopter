#ifndef PID_H
#define PID_H

#include "globals.h"

float updatePID(float targetValue, float currentValue, struct PIDdata &PIDparameters) {
    float error;
    float dTerm;
    float deltaPIDTime = MASTER_DT * CONTROL_LOOP_INTERVAL;

    error = targetValue - currentValue;

    PIDparameters.integratedError += error * deltaPIDTime;

    dTerm = (currentValue - PIDparameters.lastValue) / deltaPIDTime;

    PIDparameters.lastValue = currentValue;

    return PIDparameters.P * error +
           PIDparameters.I * PIDparameters.integratedError +
           PIDparameters.D * dTerm;
}

// void zeroIntegralError() __attribute__ ((noinline));
// void zeroIntegralError() {
//   for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
//     PID[axis].integratedError = 0;
//     PID[axis].previousPIDTime = currentTime;
//   }
// }

#endif // PID_H

