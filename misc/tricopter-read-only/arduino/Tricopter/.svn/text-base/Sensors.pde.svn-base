

void initSensors() {
  analogReference(EXTERNAL);
  Wire.begin();
  i2cSend(0x1d, 0x21, 0b01000000); // CTRL_REG2 = +-2g, BDU=1, i2c
  i2cSend(0x1d, 0x20, 0b11010111); // CTRL_REG1 = 2560Hz, Decimate by 128, enable all axis's

  for (int i = 0; i < NUM_AXES; i++) {
    gyroRaw[i] = 0;
    accelRaw[i] = 0; 

    gyroSmooth[i] = 0;
    accelSmooth[i] = 0;

    flightAngle[i] = 0;
  }
  zeroGyros();
  zeroAccels();
}

void zeroGyros() {
  int zero[50];
  for (int i = 0; i < NUM_AXES; i++) {
    for (int j = 0; j < 50; j++) {
      zero[j] = analogRead(GYRO_PORTS[i]);
    }
    gyroZero[i] = findMode(zero, 50);
  }
}


void zeroAccels() {
  accelZero[X_AXIS] = 0;
  accelZero[Y_AXIS] = 0;
  accelZero[Z_AXIS] = 0;
}

void i2cSend(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.send(reg);
  Wire.send(value);
  Wire.endTransmission(); 
}

int i2cReadAccel(int adr) {
  int r = 0;
  Wire.beginTransmission(0x1d); 
  Wire.send(adr+1); 
  Wire.endTransmission(); // HIGH
  Wire.requestFrom(0x1d, 1); 
  while(Wire.available()) { 
    r = Wire.receive(); 
  } 
  r <<= 8;
  Wire.beginTransmission(0x1d); 
  Wire.send(adr); 
  Wire.endTransmission();
  Wire.requestFrom(0x1d, 1); 
  while(Wire.available()) { 
    r += Wire.receive(); 
  } // LOW
  return r;
}


