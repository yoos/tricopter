#ifndef GYRO_H
#define GYRO_H

#include "i2c.h"

#define GYRO_ADDR 0x69 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

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
    writeTo(GYRO_ADDR, PWR_MGM, 0x00, error);
    writeTo(GYRO_ADDR, PWR_MGM, 0xFF, error); // EB, 50, 80, 7F, DE, 23, 20, FF
    writeTo(GYRO_ADDR, PWR_MGM, 0x1E, error); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
    writeTo(GYRO_ADDR, PWR_MGM, 0x00, error);

    if (error == 0)
    {
        Serial.println("ITG-3200 successfully initialized!");
    }
}

void readGyro()
{
    int gyro, error = 0;
    byte msb[1];
    byte lsb[1];
    /**************************************
        Gyro ITG-3200 I2C
        registers:
        x axis MSB = 1D, x axis LSB = 1E
        y axis MSB = 1F, y axis LSB = 20
        z axis MSB = 21, z axis LSB = 22
    **************************************/
    // Arduino Wire library (I2C)
    readFrom(GYRO_ADDR, 0x1D, 1, msb, error);   // MSB x axis
    readFrom(GYRO_ADDR, 0x1E, 1, lsb, error);   // LSB x axis

    // calculate total x axis
    gyro = (( msb[0] << 8) | lsb[0]);
    Serial.print("GX: "); Serial.print(gyro);

    // clear variables
    msb[0] = 0;
    lsb[0] = 0;
    gyro = 0;

    readFrom(GYRO_ADDR, 0x1F, 1, msb, error);   // MSB y axis
    readFrom(GYRO_ADDR, 0x20, 1, lsb, error);   // LSB y axis

    // calculate total y axis
    gyro = (( msb[0] << 8) | lsb[0]);
    Serial.print("   GY: "); Serial.print(gyro);

    // clear variables
    msb[0] = 0;
    lsb[0] = 0;
    gyro = 0;

    readFrom(GYRO_ADDR, 0x21, 1, msb, error);   // MSB z axis
    readFrom(GYRO_ADDR, 0x22, 1, lsb, error);   // LSB z axis

    // calculate z axis
    gyro = (( msb[0] << 8) | lsb[0]);
    Serial.print("   GZ: "); Serial.println(gyro);

}

#endif // GYRO_H

