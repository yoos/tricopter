/*! \file hmc5883.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for HMC5883 magnetometer class.
 *
 *  Details.
 */

#ifndef HMC5883_H
#define HMC5883_H

#include <tri/globals.h>
#include <tri/triMath.h>
#include <tri_sensors/i2c.h>

#define MAG_ADDRESS 0x1e

class HMC5883 {
    uint8_t buffer[6];
    uint16_t mRaw[3];
    float mVec[3];

public:
    HMC5883();
    void poll();   // Get bits from ITG-3200 and update gVal[].
    float* get();
    float get(int);
};

#endif // HMC5883_H

