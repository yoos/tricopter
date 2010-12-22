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
    /*****************************************
    *    ITG 3200
    *    power management set to:
    *    clock select = internal oscillator
    *         no reset, no sleep mode
    *        no standby mode
    *    sample rate to = 3Hz
    *    parameter to +/- 2000 degrees/sec
    *    low pass filter = 5Hz
    *    no interrupt
    ******************************************/
    error = sendI2C(GYRO_ADDR, PWR_MGM, 0x00);
    error = readI2C(GYRO_ADDR, 0x00, 1, gyroBuffer);
    Serial.print("Gyro Id = ");
    Serial.println(gyroBuffer[0]);

    error = sendI2C(GYRO_ADDR, PWR_MGM, 0xFF); // EB, 50, 80, 7F, DE, 23, 20, FF
    error = sendI2C(GYRO_ADDR, PWR_MGM, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
    error = sendI2C(GYRO_ADDR, PWR_MGM, 0x00);

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

    gx = ((gyroBuffer[0] << 8) | gyroBuffer[1]);
    gy = ((gyroBuffer[2] << 8) | gyroBuffer[3]);
    gz = ((gyroBuffer[4] << 8) | gyroBuffer[5]);
    
    sprintf(gyroStr, "GX: %d   GY: %d   GZ: %d", gx, gy, gz);
    Serial.print(gyroStr);
    Serial.print(10, BYTE);
}

#endif // GYRO_H

