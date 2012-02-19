/*! \file bma180.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for BMA180 accelerometer class.
 *
 *  Details.
 */

#include "bma180.h"

BMA180::BMA180(byte range, byte bw) {
    readI2C(ACCADDR, 0x00, 1, aBuffer);

    sp("BMA180 ID = ");
    spln((int) aBuffer[0]);

    // Set ee_w bit
    readI2C(ACCADDR, CTRLREG0, 1, aBuffer);
    aBuffer[0] |= 0x10;   // Bitwise OR operator to set ee_w bit.
    sendI2C(ACCADDR, CTRLREG0, aBuffer[0]);   // Have to set ee_w to write any other registers.

    // Set BandWidth
    readI2C(ACCADDR, BWTCS, 1, aBuffer);
    aBuffer[1] = bw;
    aBuffer[1] = (aBuffer[1] << 4);   // Need to shift left four bits; refer to DS p. 21.
    aBuffer[0] &= (~BWMASK);
    aBuffer[0] |= aBuffer[1];
    sendI2C(ACCADDR, BWTCS, aBuffer[0]);   // Keep tcs<3:0> in BWTCS, but write new BW.

    // Set Range
    readI2C(ACCADDR, OLSB1, 1, aBuffer);
    aBuffer[1] = range;
    aBuffer[1] = (aBuffer[1] << 1);   // Need to shift left one bit; refer to DS p. 21.
    aBuffer[0] &= (~RANGEMASK);
    aBuffer[0] |= aBuffer[1];
    sendI2C(ACCADDR, OLSB1, aBuffer[0]);   // Write new range data, keep other bits the same.

    spln("BMA180 configured!");

    for (int i=0; i<3; i++) {
        aRaw[i] = 0;
        aVal[i] = 0;
    }

    // Zero buffer.
    for (int i=0; i<READ_SIZE; i++) {
        aBuffer[i] = 0;
    }

    rkIndex = 0;
    for (int i=0; i<3; i++)
        for (int j=0; j<4; j++)
            rkVal[i][j] = 0;
}

void BMA180::poll() {
    readI2C(ACCADDR, REGADDR, READ_SIZE, aBuffer);   // Read acceleration data

    aRaw[1] = ((aBuffer[1] << 6) | (aBuffer[0] >> 2));   // Tricopter Y axis is chip X axis.
    aRaw[0] = ((aBuffer[3] << 6) | (aBuffer[2] >> 2));   // Tricopter X axis is chip Y axis. Must be negated later!
    aRaw[2] = ((aBuffer[5] << 6) | (aBuffer[4] >> 2));   // Z axis is same.

    for (int i=0; i<3; i++) {   // Convert raw values to a nice -1 to 1 range.
        float tmp;

        if (aRaw[i] < 0x2000)   // If zero to negative accel.: 0 to 2^13-1...
            tmp = -((signed) aRaw[i]);   // ...negate after casting as signed int.
        else   // If zero to positive accel.: 2^14-1 to 2^13...
            tmp = 0x3FFF - aRaw[i];   // ...subtract from 2^14-1.

        // Account for accelerometer offset, divide by maximum magnitude, and 
        // multiply by 4 to get values in range [-4g, 4g].
        switch (i) {
            case 0: aVal[i] = -tmp / 0x2000 * 4; break;   // Negated.
            case 1: aVal[i] =  tmp / 0x2000 * 4; break;
            case 2: aVal[i] =  tmp / 0x2000 * 4; break;
            default: break;
        }
    }

    // Runge-Kutta smoothing.
    #ifdef ENABLE_ACC_RK_SMOOTH
    for (int i=0; i<3; i++) {
        rkVal[i][rkIndex] = aVal[i];
        aVal[i] = (1*rkVal[i][rkIndex] +
                2*rkVal[i][(rkIndex+1)%4] +
                2*rkVal[i][(rkIndex+2)%4] +
                1*rkVal[i][(rkIndex+3)%4])/6;
    }
    rkIndex = (rkIndex + 1) % 4;   // Increment index by 1 but loop back from 3 back to 0.
    #endif // ENABLE_ACC_RK_SMOOTH
}

float* BMA180::get() {
    return aVal;   // In g's.
}

float BMA180::get(int axis) {
    return aVal[axis];
}

