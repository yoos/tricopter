#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "i2c.h"

#define ACCEL_ADDR 0x40   // BMA180 device address
#define TO_READ 6   //num of bytes we are going to read each time (two bytes for each axis)

byte accelBuffer[TO_READ] ;    //6 bytes accelBufferer for saving data read from the device
char str[512];                      //string accelBufferer to transform data before sending it to the serial port

void initAccel()
{
    int error = 0;
    readI2C(ACCEL_ADDR, 0x00, 1, accelBuffer, error);
    Serial.print("Accelerometer Id = ");
    Serial.println(accelBuffer[0]);

    if (accelBuffer[0] == 3)
    {
        sendI2C(ACCEL_ADDR, 0x0D, B0001, error);
        sendI2C(ACCEL_ADDR, 0x20, B00001000, error);
        sendI2C(ACCEL_ADDR, 0x35, B0100, error);
    }

    if (error == 0)
    {
        Serial.println("BMA180 successfully initialized!");
    }
}

void readAccel()
{
    int error = 0;
    int regAddress = 0x02;
    int x, y, z;
    
    readI2C(ACCEL_ADDR, regAddress, TO_READ, accelBuffer, error);   // Read acceleration data
    
    x = (((int)accelBuffer[1]) << 8) | accelBuffer[0];   
    y = (((int)accelBuffer[3])<< 8) | accelBuffer[2];
    z = (((int)accelBuffer[5]) << 8) | accelBuffer[4];
    
    sprintf(str, "AX: %d   AY: %d   AZ: %d", x, y, z);  
    Serial.print(str);
    Serial.print(10, BYTE);
}

#endif // ACCELEROMETER_H

