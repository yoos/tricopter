/*
 * motors.h
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */


#ifndef MOTORS_H_
#define MOTORS_H_

#include "control.h"

/* Motor Constants **************************************/
//Motor Command Bounds
#define MOTOR_MIN_COMMAND 1000
#define MOTOR_MID_COMMAND 1500
#define MOTOR_MAX_COMMAND 2000
const int MOTOR_MIN[] = {
  1060, 1060, 1060, 1060, 1060, 1060};
const int MOTOR_MAX[] = {
  1700, 1700, 1700, 1700, 1700, 1700};

//Motor Array Positions
#define NUM_MOTORS 6
#define RIGHT_TOP 0
#define RIGHT_BOTTOM 1
#define LEFT_TOP 2
#define LEFT_BOTTOM 3
#define BACK_TOP 4
#define BACK_BOTTOM 5

//Motor Output Ports
#define RIGHT_TOP_PORT 2
#define RIGHT_BOTTOM_PORT 3
#define LEFT_TOP_PORT 6
#define LEFT_BOTTOM_PORT 7
#define BACK_TOP_PORT 8
#define BACK_BOTTOM_PORT 9
const int MOTOR_PORTS[NUM_MOTORS] =
{
  RIGHT_TOP_PORT, RIGHT_BOTTOM_PORT, LEFT_TOP_PORT, LEFT_BOTTOM_PORT, BACK_TOP_PORT, BACK_BOTTOM_PORT};

void initMotors(Servo *motors);

void attachAllMotors(Servo *motors);

void detachAllMotors(Servo *motors);

// Sends commands to all motors
void commandAllMotors(int command, Servo *motors);

void killAllMotors(Servo *motors);

void updateMotors(Servo *motors, Controller *controller);

int scaleMotorCommand(float cmd, int motor);

#endif /* MOTORS_H_ */
