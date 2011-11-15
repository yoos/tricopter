#include "itg3200.h"

ITG3200::ITG3200(uint8_t range) {
    readI2C(GYRADDR, 0x00, 1, gBuffer);   // Who am I?
    
    sp("ITG-3200 ID = ");
    spln((int) gBuffer[0]);

    // Configure ITG-3200
    // Refer to DS Section 8: Register Description.
    sendI2C(GYRADDR, 0x15, 0x18);   // 00011000 -- Sample rate divider is 24(+1), so 40 Hz

    // Set Range
    readI2C(GYRADDR, 0x16, 1, gBuffer);
    gBuffer[1] = range;
    gBuffer[1] = (gBuffer[1] << 3);   // FS_SEL is on bits 4 and 3.
    gBuffer[0] |= gBuffer[1];
    sendI2C(GYRADDR, 0x16, gBuffer[0]);

    spln("ITG-3200 configured!");

    // Zero buffer.
    for (int i=0; i<READ_SIZE; i++) {
        gBuffer[i] = 0;
    }

    // For Runge-Kutta integration.
    rkIndex = 0;
    for (int i=0; i<3; i++)
        for (int j=0; j<4; j++)
            rkVal[i][j] = 0;

    for (int i=0; i<3; i++) {
        tempData[i] = 0;
        gZero[i] = 0;
        gVal[i] = 0;
        angle[i] = 0;
    }
    calibrated = false;
}

void ITG3200::Calibrate(int sampleNum) {
    for (int i=0; i<sampleNum; i++) {
        ITG3200::Poll();
        for (int j=0; j<3; j++) {
            tempData[j] = tempData[j] + gVal[j];
        }
    }

    gZero[0] = tempData[0]/sampleNum;
    gZero[1] = tempData[1]/sampleNum;
    gZero[2] = tempData[2]/sampleNum;

    spln("Gyro calibration complete!");
    sp(tempData[0]/sampleNum);
    calibrated = true;
}

void ITG3200::Poll() {
    readI2C(GYRADDR, REGADDR, READ_SIZE, gBuffer);
    
    // Shift high byte to be high 8 bits and append with low byte.
    gRaw[1] = ((gBuffer[0] << 8) | gBuffer[1]);   // Tricopter Y axis is chip X axis.
    gRaw[0] = ((gBuffer[2] << 8) | gBuffer[3]);   // Tricopter X axis is chip Y axis. Must be negated later!
    gRaw[2] = ((gBuffer[4] << 8) | gBuffer[5]);   // Z axis is same.

    // Read gyro temperature.
    readI2C(GYRADDR, TEMP_OUT, 2, gBuffer);
    temp = 35 + (((gBuffer[0] << 8) | gBuffer[1]) + 13200)/280.0;
    //sp("GT: ");
    //sp(temp);
    //sp(" ");

    // Convert raw gyro output values to rad/s.
    //
    // The constructor to this class should configure the gyro to a range of +/- 2000 deg/s over its 16-bit range:
    //     0 to 2000 deg/s: 0x0000 to 0x7fff
    //     -2000 deg/s to 0: 0x8000 to 0xffff (then wrap to 0x0000)
    // We need this to be in rad/s for the DCM.
    for (int i=0; i<3; i++) {
        float tmp;
        if (gRaw[i] >= 0x8000)   // If zero to negative rot. vel.: 2^16-1 to 2^15...
            tmp = -((signed) (0xFFFF - gRaw[i]));   // ...subtract from 2^16-1.
        else
            tmp = gRaw[i];   // Otherwise, leave it alone.
        switch (i) {
            // TODO: Arbitrary scale factor of 116/100 needs to be properly figured out.
            case 0: gVal[0] = -tmp * 2000/0x8000 * PI/180 * 116/100; break;
            case 1: gVal[1] =  tmp * 2000/0x8000 * PI/180 * 116/100; break;
            case 2: gVal[2] =  tmp * 2000/0x8000 * PI/180 * 116/100; break;
            default: break;
        }
        gVal[i] = (gVal[i] - gZero[i]);
        //if (abs(gVal[i]*1000) < 7) gVal[i] = 0;
        //sp(gVal[i]);
        //sp(" ");
    }

    // Runge-Kutta smoothing and integration.
    if (calibrated) {
        for (int i=0; i<3; i++) {
            rkVal[i][rkIndex] = gVal[i];
            gVal[i] = (1*rkVal[i][rkIndex] + 
                       2*rkVal[i][(rkIndex+1)%4] +
                       2*rkVal[i][(rkIndex+2)%4] +
                       1*rkVal[i][(rkIndex+3)%4])/6;
            //angle[i] = angle[i] + gVal[i];   // Integration.
        }
        rkIndex = (rkIndex + 1) % 4;   // Increment index by 1 but loop back from 3 back to 0.
    }
}

float* ITG3200::GetRate() {
    return gVal;
}

float ITG3200::GetRate(int axis) {
    return gVal[axis];
}

float* ITG3200::GetAngle() {
    return angle;
}

float ITG3200::GetAngle(int axis) {
    return angle[axis];
}
