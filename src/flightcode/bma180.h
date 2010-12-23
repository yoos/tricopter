#ifndef BMA180_H
#define BMA180_H

#include "i2c.h"
#include "globals.h"

#define ACCADDR 0x40   // BMA180 device address
#define READ_SIZE 6   // Number of bytes to read at a time
#define REGADDR ACCXLSB

// Calibration values
#define AXCALIB
#define AYCALIB
#define AZCALIB

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
    uint16_t aBuffer[READ_SIZE];   // Buffer to which we save data read from device
    char aStr[512];   // String buffer to organize data before sending to serial line
    uint16_t aRaw[3];
    float aVal[3];

public:
    BMA180(uint8_t, uint8_t);
    void Poll();
    float* Get();
    float Get(int);
};

#endif // BMA180_H

