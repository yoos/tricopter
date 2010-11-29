/*
 * DCMNav.c
 *
 *  Created on: Mar 31, 2010
 *      Author: ben
 */

#include "constants.h"
#include "sensors.h"


const float gravityEarth[3] = {0,0,9.8};
/*
struct DCMNavigator {
  float rotationMatrixBody2Earth[3][3];
  float accelerationEarth[3];
  float velocityEarth[3];
  float positionEarth[3];
  float gravityBody[3];
  struct PID angularRateCorrectionPI[3];  //total correction is pTerm + iTerm


};

void updateDCMNav(struct DCMNavigator nav, struct Accel accels[NUM_ACCELS], struct Gyro[NUM_GYROS]) {
  float measuredAngularRateBody[3];
  float correctedAngularRateBody[3];
  float measuredAccelerationBody[3];
  float centrifugalAccelerationBody[3];

  // Convert raw int sensor measurements to meters/sec and deg/sec floats

  // Subtract PI error correction
  // Make sure sign is right
  // Correction was computed on previous cycle.  0 on startup
  // measuredAngularRateBody = measuredAngularRateBody - (nav->angularRateCorrectionPI).cTerm

  // Apply kinematics
  // Arrange angular rates in an antisemetric matrix
  // rotMatrixBE = rotMatrixBE * antisemetric

  // Normalize the rotMatrixBE

  // Compute centrifugal acceleration
  // First compute translational velocity in body frame
  // Use velocity from last cycle and rot matrix from this cycle (is that valid?)
  // Centrifugal acceleration is anglular rate cross trans rate

  // Subtract centrifugal force (body) from measured acceleration (body)

  // Compute pure acceleration in earth grame (without gravity)
  // accelEarth = rotMatrix*accelBody - gravityEarth (0,0,9.8)

  // Integrate to get velocity (Earth)
  // velEarth = velEarth + accelEarth*dt

  // Integrate again to get position (Earth)
  // posEarth = posEarth + velEarth*dt

  // Determine direction of gravity measure by accels
  // Start with acceleration minus centrifugal force (Body)
  // Apply (very) low pass filter to extract gravity component

  // Gyro error correction is Z row of rotMatrix cross gravityBody


}
*/
