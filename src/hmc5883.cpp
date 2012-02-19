/*! \file hmc5883.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for HMC5883 magnetometer class.
 *
 *  Details.
 */

#include "hmc5883.h"

HMC5883::HMC5883() {
}

void HMC5883::poll() {
}

float* HMC5883::get() {
    return mVal;
}

float HMC5883::get(int axis) {
    return mVal[axis];
}

