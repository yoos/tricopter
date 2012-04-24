/*! \file lsm303.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for LSM303 magnetometer/accelerometer class.
 *
 *  Details.
 */

#include <tri_sensors/lsm303.h>

LSM303::LSM303() {
    readI2C(MAG_ADDRESS, 0x0f, 1, buffer);
    sp("LSM303 ID: ");
    spln((int) buffer[0]);

    // Configure magnetometer.
    readI2C(MAG_ADDRESS, 0x00, 3, buffer);

    // Data rate 75 Hz (DS p. 33).
    buffer[0] &= ~(7<<2);
    buffer[0] |= 6<<2;

    // Gain 670 LSB/gauss (DS p. 34).
    buffer[1] &= ~(7<<5);
    buffer[1] |= 3<<5;

    // Continuous-conversion mode (DS p. 34).
    buffer[2] &= ~(3);
    buffer[2] |= 0;
    writeI2C(MAG_ADDRESS, 0x00, 3, buffer);
}

void LSM303::poll() {
    // Read data.
    readI2C(MAG_ADDRESS, 0x03, 6, buffer);

    // Read in high and low bytes (2's complement).
    // Output: [0xf800 -- 0xffff] = [-2048 --   -1]
    //         [0x0000 -- 0x07ff] = [    0 -- 2047]
    mRaw[0] = ((buffer[0] << 8) | buffer[1]);   // X
    mRaw[1] = ((buffer[4] << 8) | buffer[5]);   // Y
    mRaw[2] = ((buffer[2] << 8) | buffer[3]);   // Z

    // Account for offset.
    mVec[0] = ((float) (mRaw[0] - MAG_X_MIN)) / (MAG_X_MAX - MAG_X_MIN) * 2 - 1;
    mVec[1] = ((float) (mRaw[1] - MAG_Y_MIN)) / (MAG_Y_MAX - MAG_Y_MIN) * 2 - 1;
    mVec[2] = ((float) (mRaw[2] - MAG_Z_MIN)) / (MAG_Z_MAX - MAG_Z_MIN) * 2 - 1;
    //vNorm(mVec);   // Normalize.
}

float* LSM303::get() {
    return mVec;
}

float LSM303::get(int axis) {
    return mVec[axis];
}

