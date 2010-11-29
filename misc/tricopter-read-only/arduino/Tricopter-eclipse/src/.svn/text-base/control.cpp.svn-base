/*
 * control.c
 *
 *  Created on: Mar 24, 2010
 *      Author: ben
 */
#include "control.h"
#include "nav.h"
#include "guidance.h"
#include "pid.h"

void initController(struct Controller *controller) {
  // Initialize Commands and PIDs.
}

void updateController(struct Guidance *guidance, struct Navigator *nav,
    struct Controller *controller) {

  controller->liftCmd = guidance->lift + 1000;
  controller->yawCmd = guidance->yaw / 20;

  //map(inputArray[INPUT_LIFT], LIFT_IN_MIN, LIFT_MAX, LIFT_MIN, LIFT_MAX);
  //map(inputArray[INPUT_YAW], -YAW_IN_MAX, YAW_IN_MAX, -YAW_OUT_MAX, YAW_OUT_MAX);
  //cmdPitch = map(inputArray[INPUT_PITCH], -ROLL_PITCH_IN_MAX, ROLL_PITCH_IN_MAX, -ROLL_PITCH_OUT_MAX, ROLL_PITCH_OUT_MAX);
  //cmdRoll = -map(inputArray[INPUT_ROLL],  -ROLL_PITCH_IN_MAX, ROLL_PITCH_IN_MAX, -ROLL_PITCH_OUT_MAX, ROLL_PITCH_OUT_MAX);

  controller->pitchCmd = updatePID(&(controller->pitchPID), 0, nav->flightAngles[PITCH_AXIS]);
  controller->rollCmd = updatePID(&(controller->rollPID), 0, nav->flightAngles[ROLL_AXIS]);

}
/*
if (inputButtonStatus(&input, BUTTON_LB)) {
  for (int i = 0; i <= PITCH_AXIS; i++) {
    PID[i].P += 0.025;
  }
}
if (inputButtonStatus(BUTTON_RB)) {
  for (int i = 0; i <= PITCH_AXIS; i++) {
    PID[i].P -= 0.025;
  }
}

if (inputButtonStatus(BUTTON_Y)) {
  for (int i = 0; i <= PITCH_AXIS; i++) {
    PID[i].I += 0.025;
  }
}
if (inputButtonStatus(BUTTON_B)) {
  for (int i = 0; i <= PITCH_AXIS; i++) {
    PID[i].I -= 0.025;
  }
}

  switch (flightMode) {

    case GROUND_MODE:
      //commandAllMotors(MOTOR_MIN_COMMAND);

      if (inputButtonStatus(BUTTON_START)) {
        flightMode = LIFTOFF_MODE;
      }
      if (inputButtonStatus(BUTTON_X) == 1) {
        //initSensors();
      }
      break;

    case LIFTOFF_MODE:



      //commandMotors();

      //Motor Kill Switch
      if (inputButtonStatus(BUTTON_BACK)) {
        flightMode = GROUND_MODE;
      }
      break;
  } //Switch

*/
