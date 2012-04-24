/*! \file lsm303.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for LSM303 magnetometer/accelerometer class.
 *
 *  Details.
 */

#ifndef LSM303_H
#define LSM303_H

#include <tri/globals.h>
#include <tri/triMath.h>
#include <tri_sensors/i2c.h>

#define MAG_ADDRESS 0x1e
#define ACC_ADDRESS 0x18

class LSM303 {
    uint8_t buffer[6];
    int16_t mRaw[3];   // 2's complement.
    float mVec[3];

public:
    LSM303();
    void poll();   // Get bits from ITG-3200 and update gVal[].
    float* get();
    float get(int);
};

#endif // LSM303_H

