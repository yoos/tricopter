#ifndef PID_H
#define PID_H

#include "globals.h"

// Modified from AeroQuad PID code.
struct PIDdata {
    float P, I, D;
    float lastPosition;
    float integratedError;
} PID[2];

float updatePID(float targetPosition, float currentPosition, struct PIDdata &PIDparameters) {
  float error;
  float dTerm;
  float deltaPIDTime = MASTER_DT;

  error = targetPosition - currentPosition;

  PIDparameters.integratedError += error * deltaPIDTime;
  
  dTerm = PIDparameters.D * (currentPosition - PIDparameters.lastPosition) / (deltaPIDTime * 100); // dT fix from Honk

  PIDparameters.lastPosition = currentPosition;
  
  return (PIDparameters.P * error) + (PIDparameters.I * (PIDparameters.integratedError)) + dTerm;
}

// void zeroIntegralError() __attribute__ ((noinline));
// void zeroIntegralError() {
//   for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
//     PID[axis].integratedError = 0;
//     PID[axis].previousPIDTime = currentTime;
//   }
// }

#endif // PID_H

