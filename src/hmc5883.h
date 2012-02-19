/*! \file hmc5883.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for HMC5883 magnetometer class.
 *
 *  Details.
 */

#ifndef HMC5883_H
#define HMC5883_H

#include "i2c.h"
#include "triMath.h"
#include "globals.h"

#define MAG_ADDRESS 0x1e

class HMC5883 {
    float mRaw[3];
    float mVec[3];

public:
    HMC5883();
    void poll();   // Get bits from ITG-3200 and update gVal[].
    float* get();
    float get(int);
};

#endif // HMC5883_H

