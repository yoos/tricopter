#ifndef I2C_H
#define I2C_H

#include <Wire.cpp>

void sendI2C(int device, byte address, byte val, int error) {
   Wire.beginTransmission(device);
   Wire.send(address);
   Wire.send(val);
   error = Wire.endTransmission();
}

void readI2C(int device, byte address, int num, byte buff[], int error) {
    Wire.beginTransmission(device);
    Wire.send(address);
    error = Wire.endTransmission();

    Wire.beginTransmission(device);
    Wire.requestFrom(device, num);
    int i = 0;
    while(Wire.available())   //device may send less than requested (abnormal)
    {   
        buff[i] = Wire.receive();   // receive a byte
        i++;
    }
    error = Wire.endTransmission();
}

#endif // I2C_H

