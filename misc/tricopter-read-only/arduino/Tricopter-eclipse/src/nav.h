/*
 * nav.h
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */


#ifndef NAV_H_
#define NAV_H_

#include "constants.h"
#include "sensors.h"

struct Navigator {
  float accelerations[NUM_AXES];
  float angularRates[NUM_AXES];
  float flightAngles[NUM_AXES];
  float resultant;
};

void updateNav(struct Navigator *nav, struct Accel accels[NUM_ACCELS],
    struct Gyro gyros[NUM_GYROS]);

#endif /* NAV_H_ */
