/*! \file itg3200.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for ITG3200 gyroscope class.
 *
 *  Details.
 */

#include "itg3200.h"

ITG3200::ITG3200() {
    readI2C(GYRADDR, 0x00, 1, buffer);   // Who am I?

    sp("ITG-3200 ID = ");
    spln((int) buffer[0]);

    readI2C(GYRADDR, 0x15, 2, buffer);

    // Sample rate divider is 15 + 1 = 16, so 8000 Hz / 16 = 500 Hz
    buffer[0] = 15;

    // Set FS_SEL = 3 as recommended on DS p. 24.
    // Set DLPF_CFG = 0. (8 kHz internal sample rate.)
    buffer[1] = (3 << 3);   // FS_SEL is on bits 4 and 3.

    writeI2C(GYRADDR, 0x15, 2, buffer);

    // Set to use X gyro as clock reference as recommended on DS p. 27.
    buffer[0] = 1;
    writeI2C(GYRADDR, 0x3e, 1, buffer);


    spln("ITG-3200 configured!");

    // Zero buffer.
    for (int i=0; i<6; i++) {
        buffer[i] = 0;
    }

    // Low-pass filter.
    #ifdef GYRO_LPF_DEPTH
    lpfIndex = 0;
    for (int i=0; i<3; i++)
        for (int j=0; j<GYRO_LPF_DEPTH; j++)
            lpfVal[i][j] = 0;
    #endif // GYRO_LPF_DEPTH

    for (int i=0; i<3; i++) {
        gZero[i] = 0;
        gVec[i] = 0;
        angle[i] = 0;
    }
    calibrated = false;
}

void ITG3200::calibrate(int sampleNum) {
    int32_t tmp[3];
    for (int i=0; i<3; i++) {
        tmp[i] = 0;
    }

    for (int i=0; i<sampleNum; i++) {
        ITG3200::poll();
        for (int j=0; j<3; j++) {
            tmp[j] = tmp[j] + gRaw[j];
        }
        delayMicroseconds(20);
    }

    for (int i=0; i<3; i++) {
        gZero[i] = tmp[i]/sampleNum;
    }

    spln("Gyro calibration complete!");
    calibrated = true;
}

void ITG3200::poll() {
    readI2C(GYRADDR, 0x1d, 6, buffer);

    // Shift high byte to be high 8 bits and append with low byte.
    gRaw[0] = -((buffer[2] << 8) | buffer[3]);   // Tricopter X axis is chip Y axis.
    gRaw[1] = -((buffer[0] << 8) | buffer[1]);   // Tricopter Y axis is chip X axis.
    gRaw[2] = -((buffer[4] << 8) | buffer[5]);   // Z axis is same.

    if (calibrated) {
        // Apply calibration values.
        for (int i=0; i<3; i++) {
            gRaw[i] -= gZero[i];
        }

        // Low-pass filter.
        #ifdef GYRO_LPF_DEPTH
        for (int i=0; i<3; i++) {
            lpfVal[i][lpfIndex] = gRaw[i] / GYRO_LPF_DEPTH;

            gRaw[i] = 0;
            for (int j=0; j<GYRO_LPF_DEPTH; j++) {
                gRaw[i] += lpfVal[i][(lpfIndex + j) % GYRO_LPF_DEPTH];
            }
        }
        lpfIndex = (lpfIndex + 1) % GYRO_LPF_DEPTH;   // Increment index by 1 and loop back from GYRO_LPF_DEPTH.
        #endif // GYRO_LPF_DEPTH
    }

    //sp("G( ");
    //for (int i=0; i<3; i++) {
    //    sp(gRaw[i]);
    //    sp(" ");
    //}
    //spln(")");

    // Read gyro temperature (DS p. 7).
    //readI2C(GYRADDR, TEMP_OUT, 2, buffer);
    //temp = 35 + (((buffer[0] << 8) | buffer[1]) + 13200)/280.0;

    // Convert raw gyro output values to rad/s.
    // Output: [0x0000 -- 0x7fff] = [     0 -- 32767]
    //         [0x8000 -- 0xffff] = [-32768 --    -1]
    // Range: [-2000, 2000] deg/s
    // Scale factor: 14.375 LSB / (deg/s)
    for (int i=0; i<3; i++) {
        gVec[i] = (float) gRaw[i] / 14.375 * PI/180;
    }

    //sp("G( ");
    //for (int i=0; i<3; i++) {
    //    sp(gVec[i]);
    //    sp(" ");
    //}
    //spln(")");
}

float* ITG3200::get() {
    return gVec;
}

float ITG3200::get(int axis) {
    return gVec[axis];
}

