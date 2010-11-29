/*
 * control.h
 *
 *  Created on: Mar 24, 2010
 *      Author: ben
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "pid.h"
#include "nav.h"
#include "guidance.h"

/* Axis Constants **************************************/
#define LIFT_IN_MAX 999
#define LIFT_OUT_MAX 999
#define ROLL_PITCH_IN_MAX 999
#define ROLL_PITCH_OUT_MAX 50
#define YAW_IN_MAX 999
#define YAW_OUT_MAX 50
#define YAW_CORRECTION 1.05

struct Controller {
  int liftCmd;
  int pitchCmd;
  int rollCmd;
  int yawCmd;
  struct PID pitchPID;
  struct PID rollPID;
  struct PID yawPID;
};

void initController(struct Controller *controller);

void updateController(struct Guidance *guidance, struct Navigator *nav,
    struct Controller *controller);

#endif /* CONTROL_H_ */
