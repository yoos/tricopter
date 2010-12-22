#ifndef GYRO_H
#define GYRO_H

#include "i2c.h"
#include "globals.h"

#define GYRO_ADDR 0x69 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

#define READ_SIZE 6

byte gyroBuffer[READ_SIZE];
char gyroStr[512];

//initializes the gyroscope
void initGyro()
{
    int error = 0;
    
    readI2C(GYRO_ADDR, 0x00, 1, gyroBuffer);   // Who am I?
    Serial.print("Gyro Id = ");
    Serial.println(gyroBuffer[0]);

    // Configure ITG-3200
    // Refer to datasheet Section 8: Register Description.
    sendI2C(GYRO_ADDR, 0x15, 0x18);   // 00011000 -- Sample rate divider is 24(+1)
    sendI2C(GYRO_ADDR, 0x16, 0x1A);   // 00011010 -- Internal sample rate is 1 kHz
                                      // 02, 0A, 12, 1A

    if (error == 0)
    {
        Serial.println("ITG-3200 successfully initialized!");
    }

    for (int i; i<READ_SIZE; i++) {
        gyroBuffer[i] = 0;
    }
}

void readGyro()
{
    int error = 0;
    int regAddress = 0x1D;
    int gx, gy, gz;
    /**************************************
        Gyro ITG-3200 I2C
        registers:
        x axis MSB = 1D, x axis LSB = 1E
        y axis MSB = 1F, y axis LSB = 20
        z axis MSB = 21, z axis LSB = 22
    **************************************/
    // Arduino Wire library (I2C)
    error = readI2C(GYRO_ADDR, regAddress, READ_SIZE, gyroBuffer);

    gx = ((gyroBuffer[0] << 8) | gyroBuffer[1]);   // Shift high byte to be high 8 bits and append with low byte.
    gy = ((gyroBuffer[2] << 8) | gyroBuffer[3]);
    gz = ((gyroBuffer[4] << 8) | gyroBuffer[5]);
    
    sprintf(gyroStr, "GX: %d   GY: %d   GZ: %d", gx, gy, gz);
    Serial.print(gyroStr);
    Serial.print(10, BYTE);
}

#endif // GYRO_H

