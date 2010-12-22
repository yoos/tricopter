#ifndef BMA180_H
#define BMA180_H

#include "i2c.h"
#include "globals.h"

#define DEVICE 0x40   // BMA180 device address
#define READ_SIZE 6   //num of bytes we are going to read each time (two bytes for each axis)
#define REG_ADDR 0x02

class BMA180 {
    byte aBuffer[READ_SIZE];   // Buffer to which we save data read from device
    char aString[512];   // String buffer to organize data before sending to serial line
    int eCode;
    double aVal[3];
    
public:
    BMA180();
    void Poll();
    double* Get();
    double Get(int);

};

#endif // BMA180_H

