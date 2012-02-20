/*! \file itg3200.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for ITG3200 gyroscope class.
 *
 *  Details.
 */

#ifndef ITG3200_H
#define ITG3200_H

#include "i2c.h"
#include "globals.h"

//#define ENABLE_GYRO_RK_SMOOTH   // Enable Runge-Kutta smoothing (low-pass filter)

// ITG-3200 address defines
#define GYRADDR 0x69 // gyro address, binary = 01101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define TEMP_OUT 0x1B   // 0x1B is high bit, 0x1C is low bit.
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
    uint8_t buffer[6];   // Buffer for general use. Increase size as needed.
    char gStr[512];

    uint16_t gRaw[3];   // Raw bits received from ITG-3200
    float gVec[3];   // [-2000, 2000] deg/s mapped to [-1, 1].
    bool calibrated;   // Disable integration until calibration finishes.
    float tempData[3];   // Temporary storage of calibration data
    float gZero[3];   // Zero values
    float angle[3];   // Calculated angles of rotation
    float temp;

    int rkIndex;
    float rkVal[3][4];   // Four Runge-Kutta integrator values for each of three axes

public:
    ITG3200();
    void calibrate(int);
    void poll();   // Get bits from ITG-3200 and update gVec[].
    float* get();
    float get(int);
};

#endif // ITG3200_H

