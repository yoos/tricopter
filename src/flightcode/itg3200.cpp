#include "itg3200.h"

ITG3200::ITG3200(uint8_t range) {
    readI2C(GYRADDR, 0x00, 1, gBuffer);   // Who am I?
    
    Serial.print("ITG-3200 ID = ");
    Serial.println((int) gBuffer[0]);

    // Configure ITG-3200
    // Refer to DS Section 8: Register Description.
    sendI2C(GYRADDR, 0x15, 0x18);   // 00011000 -- Sample rate divider is 24(+1), so 40 Hz

    // Set Range
    readI2C(GYRADDR, 0x16, 1, gBuffer);
    gBuffer[1] = range;
    gBuffer[1] = (gBuffer[1] << 3);   // FS_SEL is on bits 4 and 3.
    gBuffer[0] |= gBuffer[1];
    sendI2C(GYRADDR, 0x16, gBuffer[0]);

    Serial.println("ITG-3200 configured!");

    // Zero buffer.
    for (int i=0; i<READ_SIZE; i++) {
        gBuffer[i] = 0;
    }

    rkIndex = 0;
    rkVal = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
    angle = {0, 0, 0};
}

void ITG3200::Poll() {
    readI2C(GYRADDR, REGADDR, READ_SIZE, gBuffer);

    gRaw[0] = ((gBuffer[0] << 8) | gBuffer[1]);   // Shift high byte to be high 8 bits and append with low byte.
    gRaw[1] = ((gBuffer[2] << 8) | gBuffer[3]);
    gRaw[2] = ((gBuffer[4] << 8) | gBuffer[5]);

    // Convert raw values to a nice -1 to 1 range.
    for (int i=0; i<3; i++) {
        float tmp;
        if (gRaw[i] >= 0x8000)   // If zero to negative rot. vel.: 2^16-1 to 2^15...
            tmp = -((signed) (0xFFFF - gRaw[i]));   // ...subtract from 2^16-1.
        else
            tmp = gRaw[i];
        switch (i) {
            case 0: gVal[0] = (tmp - GXOFFSET)/0x8000; break;   // Divide by maximum magnitude.
            case 1: gVal[1] = (tmp - GYOFFSET)/0x8000; break;
            case 2: gVal[2] = (tmp - GZOFFSET)/0x8000; break;
            default: break;
        }
    }

    #ifdef DEBUG
        Serial.print("GX: "); Serial.print(gVal[0]);
        Serial.print("   GY: "); Serial.print(gVal[1]);
        Serial.print("   GZ: "); Serial.println(gVal[2]);
    #endif

    ITG3200::UpdateRK();   // Runge-Kutta integration
}

// Smooth data.
void ITG3200::UpdateRK() {
    for (int i=0; i<3; i++) {
        rkVal[i][rkIndex] = gVal[i]*2000 * SYSINTRV/1000 * 8/7;   // [-1,1] mapped to [-2000,2000] and system run interval accounted for. 8/7 gain, but don't know why.
        // Runge-Kutta integration
        angle[i] = angle[i] + (1*rkVal[i][rkIndex] + 
                               2*rkVal[i][(rkIndex+1)%4] +
                               2*rkVal[i][(rkIndex+2)%4] +
                               1*rkVal[i][(rkIndex+3)%4])/6;
    }
    rkIndex = (rkIndex + 1) % 4;   // Increment index by 1 but loop back from 3 back to 0.

    // Serial.print("LX: "); Serial.print(angle[0]);
    // Serial.print("   LY: "); Serial.print(angle[1]);
    // Serial.print("   LZ: "); Serial.println(angle[2]);
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
