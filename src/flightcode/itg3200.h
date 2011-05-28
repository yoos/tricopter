#ifndef GYRO_H
#define GYRO_H

#include "i2c.h"
#include "globals.h"

#define READ_SIZE 6   // Number of bytes to read each time
#define REGADDR 0x1D

// Calibration values
#define GXOFFSET 312
#define GYOFFSET -90
#define GZOFFSET 180

// ITG-3200 address defines
#define GYRADDR 0x69 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E


/* WIRING GUIDE
 * 10 kOhm pull-ups on I2C lines.
 * SCL -> SCL
 * SDA -> SDA
 * CLK -> GND
 * INT -- not connected
 * GND -> GND
 * VIO -> 3.3V
 * VDD -> 3.3V
 */


class ITG3200 {
    uint16_t gBuffer[READ_SIZE];
    char gStr[512];

    uint16_t gRaw[3];   // Raw bits received from ITG-3200
    float gVal[3];   // Bits mapped to [-1,1]

    int rkIndex;
    float rkVal[3][4];   // Four Runge-Kutta integrator values for each of three axes
    float angle[3];   // Calculated angles of rotation

    void UpdateRK();   // Update Runge-Kutta integrator values and calculate angle[].

public:
    ITG3200(uint8_t);
    void Poll();   // Get bits from ITG-3200 and update gVal[].
    float* GetRate();
    float GetRate(int);
    float* GetAngle();
    float GetAngle(int);
};

#endif // GYRO_H

