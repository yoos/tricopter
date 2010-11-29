void initMotors() {
  attachAllMotors();
  commandAllMotors(MOTOR_MIN_COMMAND);
}

void attachAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(MOTOR_PORTS[i]);
  }
}

void detachAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].detach();
  }
}

void commandMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].write(motorCommands[i]);
  }
}

// Sends commands to all motors
void commandAllMotors(int command) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorCommands[i] = command;
  }
  commandMotors();
}

void killMotors() {
  commandAllMotors(MOTOR_MIN_COMMAND);
}






int scaleMotorCommand(float cmd, int motor) {
  return map(int(cmd), 930, 2070, MOTOR_MIN[motor], MOTOR_MAX[motor]);
}
