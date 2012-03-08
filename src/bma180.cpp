/*! \file bma180.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for BMA180 accelerometer class.
 *
 *  Details.
 */

#include "bma180.h"

BMA180::BMA180() {
    readI2C(ACCADDR, 0x00, 1, buffer);

    sp("BMA180 ID = ");
    spln((int) buffer[0]);

    // Set ee_w bit
    readI2C(ACCADDR, CTRLREG0, 1, buffer);
    buffer[0] |= 0x10;   // Bitwise OR operator to set ee_w bit.
    sendI2C(ACCADDR, CTRLREG0, buffer[0]);   // Have to set ee_w to write any other registers.

    // Set range.
    readI2C(ACCADDR, OLSB1, 1, buffer);
    buffer[0] &= (~0x0e);   // Clear old ACC_RANGE bits.
    buffer[0] |= (ACC_RANGE << 1);   // Need to shift left one bit; refer to DS p. 21.
    sendI2C(ACCADDR, OLSB1, buffer[0]);   // Write new ACC_RANGE data, keep other bits the same.

    // Set ADC resolution (DS p. 8).
    res = 0.000125;                        // [   -1,   1] g
    if      (ACC_RANGE == 1) res *= 1.5;   // [ -1.5, 1.5] g
    else if (ACC_RANGE == 2) res *= 2;     // [   -2,   2] g
    else if (ACC_RANGE == 3) res *= 3;     // [   -3,   3] g
    else if (ACC_RANGE == 4) res *= 4;     // [   -4,   4] g
    else if (ACC_RANGE == 5) res *= 8;     // [   -8,   8] g
    else if (ACC_RANGE == 6) res *= 16;    // [  -16,  16] g

    // Set bandwidth.
    //     ACC_BW  bandwidth (Hz)
    //          0              10
    //          1              20
    //          2              40
    //          3              75
    //          4             150
    //          5             300
    //          6             600
    //          7            1200
    readI2C(ACCADDR, BWTCS, 1, buffer);
    buffer[0] &= (~0xf0);   // Clear bandwidth bits <7:4>.
    buffer[0] |= (ACC_BW << 4);   // Need to shift left four bits; refer to DS p. 21.
    sendI2C(ACCADDR, BWTCS, buffer[0]);   // Keep tcs<3:0> in BWTCS, but write new ACC_BW.

    // Set mode_config to 0x01 (ultra low noise mode, DS p. 28).
    //readI2C(ACCADDR, 0x30, 1, buffer);
    //buffer[0] &= (~0x03);   // Clear mode_config bits <1:0>.
    //buffer[0] |= 0x01;
    //sendI2C(ACCADDR, 0x30, buffer[0]);

    spln("BMA180 configured!");

    for (int i=0; i<3; i++) {
        aRaw[i] = 0;
        aVec[i] = 0;
    }

    // Zero buffer.
    for (int i=0; i<6; i++) {
        buffer[i] = 0;
    }

    // Low-pass filter.
    lpfIndex = 0;
    for (int i=0; i<3; i++)
        for (int j=0; j<ACC_LPF_DEPTH; j++)
            lpfVal[i][j] = 0;
}

void BMA180::poll() {
    // Read data.
    readI2C(ACCADDR, 0x02, 6, buffer);

    aRaw[0] = ((buffer[3] << 6) | (buffer[2] >> 2));   // Tricopter X axis is chip Y axis.
    aRaw[1] = ((buffer[1] << 6) | (buffer[0] >> 2));   // Tricopter Y axis is chip X axis.
    aRaw[2] = ((buffer[5] << 6) | (buffer[4] >> 2));   // Z axis is same.

    // Read accelerometer temperature.
    //readI2C(ACCADDR, 0x08, 1, buffer);
    //temp = -40 + 0.5 * buffer[0];   // 0.5 K/LSB

    // Convert raw values to multiples of gravitational acceleration.
    // Output: [0x1fff -- 0x0000] = [-8191 --    0]
    //         [0x3fff -- 0x2000] = [    1 -- 8192]
    // ADC resolution varies depending on setup. See DS p. 27 or the
    // constructor of this class.
    //aVec[0] = (float) -((int) aRaw[0]) * res;
    //aVec[1] = (float)  ((int) aRaw[1]) * res;
    //aVec[2] = (float)  ((int) aRaw[2]) * res;

    // DEPRECATED ADC CONVERSION CODE
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
    #ifdef ACC_LPF_DEPTH
    for (int i=0; i<3; i++) {
        lpfVal[i][lpfIndex] = aVec[i] / ACC_LPF_DEPTH;

        aVec[i] = 0;
        for (int j=0; j<ACC_LPF_DEPTH; j++) {
            aVec[i] += lpfVal[i][(lpfIndex + j) % ACC_LPF_DEPTH];
        }

        // DEPRECATED
        //aVec[i] = (1*lpfVal[i][lpfIndex] +
        //        2*lpfVal[i][(lpfIndex+1)%4] +
        //        2*lpfVal[i][(lpfIndex+2)%4] +
        //        1*lpfVal[i][(lpfIndex+3)%4])/6;
    }
    lpfIndex = (lpfIndex + 1) % ACC_LPF_DEPTH;   // Increment index by 1 and loop back from ACC_LPF_DEPTH.
    #endif // ACC_LPF_DEPTH
}

float* BMA180::get() {
    return aVec;   // In g's.
}

float BMA180::get(int axis) {
    return aVec[axis];
}

