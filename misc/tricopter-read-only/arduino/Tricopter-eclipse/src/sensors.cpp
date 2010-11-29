#include <WProgram.h>
#include <Wire.h>
#include "sensors.h"
#include "utils.h"

void initAccels(struct Accel accels[NUM_ACCELS]) {
  Wire.begin();
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's

  for (int i = 0; i < NUM_ACCELS; i++) {
    accels[i].i2c_addr = ACCEL_ADDRS[i];
  }
  zeroAccels(accels);
}

void updateAccel(struct Accel *accel) {
  accel->rawValue = i2cReadAccel(accel->i2c_addr) - accel->zero;
//  accel->value = accel->convert(accel->rawValue);
}
void updateAccels(struct Accel accels[NUM_ACCELS]) {
  for (int i = 0; i < NUM_ACCELS; i++) {
    updateAccel(&(accels[i]));
  }
}

void zeroAccels(struct Accel accels[NUM_ACCELS]) {
  for (int i = 0; i < NUM_ACCELS; i++) {
    accels[i].zero = 0;
  }
}

void initGyros(struct Gyro gyros[NUM_GYROS]) {
  analogReference(EXTERNAL);
  for (int i = 0; i < NUM_GYROS; i++) {
    gyros[i].pin = GYRO_PORTS[i];
  }
  zeroGyros(gyros);
}

void updateGyro(struct Gyro *gyro) {
  gyro->rawValue = analogRead(gyro->pin) - gyro->zero;
//  gyro->value = gyro->convert(gyro->rawValue);
}

void updateGyros(struct Gyro gyros[NUM_GYROS]) {
  for (int i = 0; i < NUM_GYROS; i++) {
    updateGyro(&(gyros[i]));
  }
}

void zeroGyros(struct Gyro gyros[NUM_GYROS]) {
  int zero[50];
  for (int i = 0; i < NUM_GYROS; i++) {
    for (int j = 0; j < 50; j++) {
      zero[j] = analogRead(gyros[i].pin);
    }
    gyros[i].zero = findMode(zero, 50);
  }
}

void initSensors(struct Accel accels[NUM_ACCELS], struct Gyro gyros[NUM_GYROS]) {
  initAccels(accels);
  initGyros(gyros);
}


void i2cSend(char address, char reg,  char value) {
  Wire.beginTransmission(address);
  Wire.send(reg);
  Wire.send(value);
  Wire.endTransmission();
}

int i2cReadAccel(int adr) {
  int r = 0;
  Wire.beginTransmission(0x1d);
  Wire.send(adr + 1);
  Wire.endTransmission(); // HIGH
  Wire.requestFrom(0x1d, 1);
  while (Wire.available()) {
    r = Wire.receive();
  }
  r <<= 8;
  Wire.beginTransmission(0x1d);
  Wire.send(adr);
  Wire.endTransmission();
  Wire.requestFrom(0x1d, 1);
  while (Wire.available()) {
    r += Wire.receive();
  } // LOW
  return r;
}

