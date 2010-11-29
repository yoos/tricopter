/*
 * nav.c
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */
#include <WProgram.h>
#include "sensors.h"
#include "nav.h"

void updateNav(struct Navigator *nav, struct Accel accels[NUM_ACCELS],
    struct Gyro gyros[NUM_GYROS]) {
  // Apply simple low pass filter to sensor inputs
  for (int i = 0; i < NUM_AXES; i++) {
    nav->accelerations[i] = nav->accelerations[i] * .75 + accels[i].value * .25;
    nav->angularRates[i] = nav->angularRates[i] * .75 + gyros[i].value * .25;
  }

  // Compute Resultant
  nav->resultant = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    nav->resultant += nav->accelerations[i] * nav->accelerations[i];
  }
  nav->resultant = sqrt(nav->resultant) * 10 / 1024;

  // Determine flight angle purely from accelerations
  nav->flightAngles[PITCH_AXIS] = atan2(-nav->accelerations[Y_AXIS],
      nav->accelerations[Z_AXIS]) * (180 / PI) + 180;
  nav->flightAngles[ROLL_AXIS] = atan2(-nav->accelerations[X_AXIS],
      nav->accelerations[Z_AXIS]) * (180 / PI) + 180;
  for (int i = 0; i < NUM_AXES; i++) {
    if (nav->flightAngles[i] > 180.0) {
      nav->flightAngles[i] -= 360.0;
    }
  }

  //Simple integration to determine flight angle (times 10)
  //flightAngle[ROLL_AXIS] += gyroSmooth[ROLL_AXIS] * .3538 * dt * 10;
  //flightAngle[PITCH_AXIS] += gyroSmooth[PITCH_AXIS] * .3538 * dt * 10;
  //flightAngle[YAW_AXIS] += gyroSmooth[YAW_AXIS] * .9765625 * dt * 10;
}
