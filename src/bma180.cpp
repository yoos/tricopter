/*! \file bma180.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for BMA180 accelerometer class.
 *
 *  Details.
 */

#include "bma180.h"

BMA180::BMA180(uint8_t range, uint8_t bw) {
    readI2C(ACCADDR, 0x00, 1, buffer);

    sp("BMA180 ID = ");
    spln((int) buffer[0]);

    // Set ee_w bit
    readI2C(ACCADDR, CTRLREG0, 1, buffer);
    buffer[0] |= 0x10;   // Bitwise OR operator to set ee_w bit.
    sendI2C(ACCADDR, CTRLREG0, buffer[0]);   // Have to set ee_w to write any other registers.

    // Set range.
    readI2C(ACCADDR, OLSB1, 1, buffer);
    buffer[1] = range;
    buffer[1] = (buffer[1] << 1);   // Need to shift left one bit; refer to DS p. 21.
    buffer[0] &= (~RANGEMASK);
    buffer[0] |= buffer[1];
    sendI2C(ACCADDR, OLSB1, buffer[0]);   // Write new range data, keep other bits the same.

    if      (range == 0x000) res = 0.13;   // [   -1,   1] g
    else if (range == 0x001) res = 0.19;   // [ -1.5, 1.5] g
    else if (range == 0x010) res = 0.25;   // [   -2,   2] g
    else if (range == 0x011) res = 0.38;   // [   -3,   3] g
    else if (range == 0x100) res = 0.50;   // [   -4,   4] g
    else if (range == 0x101) res = 0.99;   // [   -8,   8] g
    else if (range == 0x110) res = 1.98;   // [  -16,  16] g

    // Set bandwidth.
    //         bw  bandwidth (Hz)
    //     0x0000              10
    //     0x0001              20
    //     0x0010              40
    //     0x0011              75
    //     0x0100             150
    //     0x0101             300
    //     0x0110             600
    //     0x0111            1200
    readI2C(ACCADDR, BWTCS, 1, buffer);
    buffer[1] = bw;
    buffer[1] = (buffer[1] << 4);   // Need to shift left four bits; refer to DS p. 21.
    buffer[0] &= (~BWMASK);
    buffer[0] |= buffer[1];
    sendI2C(ACCADDR, BWTCS, buffer[0]);   // Keep tcs<3:0> in BWTCS, but write new BW.

    spln("BMA180 configured!");

    for (int i=0; i<3; i++) {
        aRaw[i] = 0;
        aVec[i] = 0;
    }

    // Zero buffer.
    for (int i=0; i<6; i++) {
        buffer[i] = 0;
    }

    rkIndex = 0;
    for (int i=0; i<3; i++)
        for (int j=0; j<4; j++)
            rkVal[i][j] = 0;
}

void BMA180::poll() {
    // Read data.
    readI2C(ACCADDR, 0x02, 6, buffer);

    aRaw[1] = ((buffer[1] << 6) | (buffer[0] >> 2));   // Tricopter Y axis is chip X axis.
    aRaw[0] = ((buffer[3] << 6) | (buffer[2] >> 2));   // Tricopter X axis is chip Y axis. Must be negated later!
    aRaw[2] = ((buffer[5] << 6) | (buffer[4] >> 2));   // Z axis is same.

    // Convert raw values to multiples of gravitational acceleration.
    // Output: [0x1fff -- 0x0000] = [-8191 --    0]
    //         [0x3fff -- 0x2000] = [    1 -- 8192]
    // ADC resolution varies depending on setup. See DS p. 27 or the
    // constructor of this class.
    for (int i=0; i<3; i++) {
        float tmp;

        if (aRaw[i] < 0x2000)
            tmp = -((signed) aRaw[i]);
        else
            tmp = 0x4000 - aRaw[i];

        aVec[i] = tmp * res;
    }
    aVec[0] *= -1;   // Negated.

    // Runge-Kutta smoothing.
    #ifdef ENABLE_ACC_RK_SMOOTH
    for (int i=0; i<3; i++) {
        rkVal[i][rkIndex] = aVec[i];
        aVec[i] = (1*rkVal[i][rkIndex] +
                2*rkVal[i][(rkIndex+1)%4] +
                2*rkVal[i][(rkIndex+2)%4] +
                1*rkVal[i][(rkIndex+3)%4])/6;
    }
    rkIndex = (rkIndex + 1) % 4;   // Increment index by 1 but loop back from 3 back to 0.
    #endif // ENABLE_ACC_RK_SMOOTH
}

float* BMA180::get() {
    return aVec;   // In g's.
}

float BMA180::get(int axis) {
    return aVec[axis];
}

