#include <WProgram.h>
#include <Servo.h>
#include "motors.h"
#include "control.h"

void initMotors(Servo *motors) {
  attachAllMotors(motors);
  commandAllMotors(MOTOR_MIN_COMMAND, motors);
}

void attachAllMotors(Servo *motors) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(MOTOR_PORTS[i]);
  }
}

void detachAllMotors(Servo *motors) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].detach();
  }
}

// Sends commands to all motors
void commandAllMotors(int command, Servo *motors) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].write(command);
  }
}

void killAllMotors(Servo *motors) {
  commandAllMotors(MOTOR_MIN_COMMAND, motors);
}

void updateMotors(Servo *motors, Controller *controller) {
  int lift = controller->liftCmd;
  int pitch = controller->pitchCmd;
  int roll = controller->rollCmd;
  int yaw = controller->yawCmd;

  motors[RIGHT_TOP].write(int(lift + yaw + pitch/3.0 + roll/1.73205));
  motors[RIGHT_BOTTOM].write(int(lift - yaw + pitch/3.0 + roll/1.73205));
  motors[LEFT_TOP].write(int(lift + yaw - pitch/3.0 - roll/1.73205));
  motors[LEFT_BOTTOM].write(int(lift - yaw - pitch/3.0 - roll/1.73205));
  motors[BACK_TOP].write(int(lift + yaw + 2.0*pitch/3.0));
  motors[BACK_BOTTOM].write(int(lift - yaw + 2.0*pitch/3.0));
}


int scaleMotorCommand(float cmd, int motor) {
  return map(int(cmd), 930, 2070, MOTOR_MIN[motor], MOTOR_MAX[motor]);
}
