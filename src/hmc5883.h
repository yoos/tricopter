/*! \file itg3200.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for ITG3200 gyroscope class.
 *
 *  Details.
 */

#ifndef HMC5883_H
#define HMC5883_H

#include "i2c.h"
#include "globals.h"

//#define ENABLE_GYRO_RK_SMOOTH   // Enable Runge-Kutta smoothing (low-pass filter)

class HMC5883 {
    float mVal[3];

public:
    HMC5883();
    void poll();   // Get bits from ITG-3200 and update gVal[].
    float* get();
    float get(int);
};

#endif // HMC5883_H

