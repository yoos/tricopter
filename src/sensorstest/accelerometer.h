#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "i2c.h"

#define ACCEL_ADDR 0x40   // BMA180 device address
#define TO_READ 6   //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];                      //string buffer to transform data before sending it to the serial port

void initAccel()
{
    int error = 0;
    readFrom(ACCEL_ADDR, 0x00, 1, buff, error);
    Serial.print("Accelerometer Id = ");
    Serial.println(buff[0]);

    if (buff[0] == 3)
    {
        writeTo(ACCEL_ADDR, 0x0D, B0001, error);
        writeTo(ACCEL_ADDR, 0x20, B00001000, error);
        writeTo(ACCEL_ADDR, 0x35, B0100, error);
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
    
    readFrom(ACCEL_ADDR, regAddress, TO_READ, buff, error);   // Read acceleration data
    
    x = (((int)buff[1]) << 8) | buff[0];   
    y = (((int)buff[3])<< 8) | buff[2];
    z = (((int)buff[5]) << 8) | buff[4];
    
    sprintf(str, "AX: %d   AY: %d   AZ: %d", x, y, z);  
    Serial.print(str);
    Serial.print(10, BYTE);
}

#endif // ACCELEROMETER_H

