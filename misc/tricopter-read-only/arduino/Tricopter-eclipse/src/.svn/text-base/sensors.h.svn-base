/*
 * sensors.h
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */


#ifndef SENSORS_H_
#define SENSORS_H_

#include "constants.h"

/* Sensor Constants *************************************/


// Gyro Analog ADC PIN constants
#define NUM_GYROS 3
#define ROLL_GYRO_PORT 0
#define PITCH_GYRO_PORT 1
#define YAW_GYRO_PORT 2
const int GYRO_PORTS[NUM_GYROS] = { ROLL_GYRO_PORT, PITCH_GYRO_PORT,
    YAW_GYRO_PORT };

// Accel Register Constants
#define NUM_ACCELS 3
#define X_ACCEL_ADDR 0x2A
#define Y_ACCEL_ADDR 0x28
#define Z_ACCEL_ADDR 0x2C
const int ACCEL_ADDRS[NUM_ACCELS] = { X_ACCEL_ADDR, Y_ACCEL_ADDR, Z_ACCEL_ADDR };

struct Accel {
  int rawValue;
  int zero;
  float value;
  float (*convert)(int);
  int i2c_addr;
};

struct Gyro {
  int rawValue;
  int zero;
  float value;
//  float (*convert)(int);
  int pin;
};

void initAccels(struct Accel accel[NUM_ACCELS]);
void updateAccels(struct Accel accel[NUM_ACCELS]);
void updateAccel(struct Accel *accel);
void zeroAccels(struct Accel accels[NUM_ACCELS]);

void initGyros(struct Gyro gyros[NUM_GYROS]);
void updateGyros(struct Gyro gyro[NUM_GYROS]);
void updateGyro(struct Gyro *gyro);
void zeroGyros(struct Gyro gyros[NUM_GYROS]);

void initSensors(struct Accel accels[NUM_ACCELS], struct Gyro gyros[NUM_GYROS]);


void i2cSend(char address, char reg, char value);
int i2cReadAccel(int adr);

#endif /* SENSORS_H_ */
