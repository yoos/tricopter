/*! \file lsm303.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for LSM303 magnetometer/accelerometer class.
 *
 *  Details.
 */

#include "lsm303.h"

LSM303::LSM303() {
    readI2C(MAG_ADDRESS, 0x0f, 1, buffer);
    sp("LSM303 ID: ");
    spln((int) buffer[0]);

    // Set magnetometer data rate to 75 Hz (DS p. 33).
    readI2C(MAG_ADDRESS, 0x00, 1, buffer);
    buffer[0] &= ~(7<<2);   // Clear out relevant register.
    buffer[0] |= 6<<2;
    sendI2C(MAG_ADDRESS, 0x00, buffer[0]);

    // Set magnetometer operation mode to continuous-conversion (DS p. 34).
    readI2C(MAG_ADDRESS, 0x02, 1, buffer);
    buffer[0] &= ~(3);   // Clear out relevant register.
    buffer[0] |= 0;
    sendI2C(MAG_ADDRESS, 0x02, buffer[0]);
}

void LSM303::poll() {
    // Read data.
    readI2C(MAG_ADDRESS, 0x03, 6, buffer);

    // Read in high and low bytes.
    mRaw[0] = ((buffer[0] << 8) | buffer[1]);   // X
    mRaw[1] = ((buffer[4] << 8) | buffer[5]);   // Y
    mRaw[2] = ((buffer[2] << 8) | buffer[3]);   // Z

    // Convert raw outputs to vector values.
    // Output: [0xf800 -- 0xffff] = [-2048 --   -1]
    //         [0x0000 -- 0x07ff] = [    0 -- 2047]
    for (int i=0; i<3; i++) {
        float tmp;

        if (mRaw[i] >= 0xf800)
            tmp = -((signed) (0x10000 - mRaw[i]));
        else
            tmp = mRaw[i];

        mVec[i] = (int16_t) mRaw[i];
    }
    //vNorm(mVec);   // Normalize.
}

float* LSM303::get() {
    return mVec;
}

float LSM303::get(int axis) {
    return mVec[axis];
}

