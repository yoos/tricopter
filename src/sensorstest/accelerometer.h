#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "i2c.h"

#define ACCEL_ADDR 0x40   // BMA180 device address
#define READ_SIZE 6   //num of bytes we are going to read each time (two bytes for each axis)

byte accelBuffer[READ_SIZE] ;    //6 bytes buffer for saving data read from the device
char str[512];                      //string buffer to transform data before sending it to the serial port

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

    for (int i; i<READ_SIZE; i++) {
        accelBuffer[i] = 0;
    }
}

void readAccel()
{
    int error = 0;
    int regAddress = 0x02;
    int ax, ay, az;
    
    readI2C(ACCEL_ADDR, regAddress, READ_SIZE, accelBuffer, error);   // Read acceleration data
    
    ax = (((int)accelBuffer[1]) << 8) | accelBuffer[0];   
    ay = (((int)accelBuffer[3]) << 8) | accelBuffer[2];
    az = (((int)accelBuffer[5]) << 8) | accelBuffer[4];
    
    sprintf(str, "AX: %d   AY: %d   AZ: %d", ax, ay, az);  
    Serial.print(str);
    Serial.print(10, BYTE);
}

#endif // ACCELEROMETER_H

