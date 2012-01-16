/*! \file i2c.h
 *  \author Soo-Hyun Yoo
 *  \brief Functions to make I2C communication easier.
 *
 *  Details.
 */

#ifndef I2C_H
#define I2C_H

#include <Wire.h>

int sendI2C(int device, byte address, byte val) {
   Wire.beginTransmission(device);
   Wire.send(address);
   Wire.send(val);
   return Wire.endTransmission();
}

int readI2C(int device, byte address, int num, byte buff[]) {
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

