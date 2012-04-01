/*! \file lsm303dlm.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for LSM303DLM magnetometer/accelerometer class.
 *
 *  Details.
 */

#ifndef LSM303DLM_H
#define LSM303DLM_H

#include "i2c.h"
#include "triMath.h"
#include "globals.h"

#define MAG_ADDRESS 0x1e

class LSM303DLM {
    uint8_t buffer[6];
    uint16_t mRaw[3];
    float mVec[3];

public:
    LSM303DLM();
    void poll();   // Get bits from ITG-3200 and update gVal[].
    float* get();
    float get(int);
};

#endif // LSM303DLM_H

