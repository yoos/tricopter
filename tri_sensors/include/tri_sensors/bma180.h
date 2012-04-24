/*! \file bma180.h
 *  \author Soo-Hyun Yoo
 *  \brief Header file for BMA180 accelerometer class.
 *
 *  Details.
 */

#ifndef BMA180_H
#define BMA180_H

#include <tri/globals.h>
#include <tri_sensors/i2c.h>

//#define ACC_LPF_DEPTH 8   // Enable low-pass filter.

#define ACC_RANGE 4
#define ACC_BW 5

#define ACCADDR 0x40   // BMA180 device address.

// BMA180 address defines
#define ID 0x00
#define VERSION 0x01
#define ACCXLSB 0x02
#define ACCXMSB 0x03
#define ACCYLSB 0x04
#define ACCYMSB 0x05
#define ACCZLSB 0x06
#define ACCZMSB 0x07
#define ACCTEMP 0x08
#define STATREG1 0x09
#define STATREG2 0x0A
#define STATREG3 0x0B
#define STATREG4 0x0C
#define CTRLREG0 0x0D
#define CTRLREG1 0x0E
#define CTRLREG2 0x0F
#define BWTCS 0x20
#define CTRLREG3 0x21
#define LOWTH 0x29
#define tco_y 0x2F
#define tco_z 0x30
#define OLSB1 0x35


class BMA180 {
    float res;   // ADC resolution (mg/LSB) based on construction inputs.

    uint8_t buffer[6];   // Buffer to which we save data read from device
    int16_t aRaw[3];   // Raw digital values.
    float aVec[3];   // In g's
    float temp;   // Temperature.

    #ifdef ACC_LPF_DEPTH
    int lpfIndex;
    int16_t lpfVal[3][ACC_LPF_DEPTH];   // Low-pass filter values for each of three axes
    #endif // ACC_LPF_DEPTH

public:
    BMA180();
    void poll();
    float* get();
    float get(int);
};

#endif // BMA180_H

