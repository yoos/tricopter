#ifndef I2C_H
#define I2C_H

#include <Wire.cpp>

int sendI2C(int device, uint8_t address, uint8_t val) {
   Wire.beginTransmission(device);
   Wire.send(address);
   Wire.send(val);
   return Wire.endTransmission();
}

int readI2C(int device, uint8_t address, int num, uint8_t buff[]) {
    int eCode;

    Wire.beginTransmission(device);
    Wire.send(address);
    eCode = Wire.endTransmission();

    Wire.beginTransmission(device);
    Wire.requestFrom(device, num);
    int i = 0;
    while(Wire.available())   //device may send less than requested (abnormal)
    {   
        buff[i] = Wire.receive();   // receive a uint8_t
        i++;
    }
    eCode = Wire.endTransmission();

    return eCode;
}

#endif // I2C_H

