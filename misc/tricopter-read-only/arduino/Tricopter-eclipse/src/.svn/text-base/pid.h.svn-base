/*
 * pid.h
 *
 *  Created on: Mar 22, 2010
 *      Author: ben
 */

#ifndef PID_H_
#define PID_H_

struct PID {
  float P_GAIN, I_GAIN, D_GAIN;
  float pTerm, iTerm, dTerm, cTerm;  //cTerm is combined term -> pTerm+iTerm+dTerm
  float lastPosition;
  float integratedError;
  float windupGaurd;
};

void initPID(struct PID *p, float P_GAIN, float I_GAIN, float D_GAIN,
    float windupGaurd);

float updatePID(struct PID *p, float targetPosition, float currentPosition);

void zeroPIDIntegratedError(struct PID *p);
#endif /* PID_H_ */
