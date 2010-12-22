#include "itg3200.h"

ITG3200::ITG3200() {
    readI2C(GYRADDR, 0x00, 1, gBuffer);   // Who am I?
    
    #ifdef DEBUG
        Serial.print("Gyro ID = ");
        Serial.println(gBuffer[0]);
    #endif

    // Configure ITG-3200
    // Refer to DS Section 8: Register Description.
    sendI2C(GYRADDR, 0x15, 0x18);   // 00011000 -- Sample rate divider is 24(+1)
    sendI2C(GYRADDR, 0x16, 0x1A);   // 00011010 -- Internal sample rate is 1 kHz
                                      // 02, 0A, 12, 1A are Reserved, Reserved, Reserved, and 2000 deg/s

    #ifdef DEBUG
        Serial.println("ITG-320 successfully initialized!");
    #endif

    // Zero buffer.
    for (int i=0; i<READ_SIZE; i++) {
        gBuffer[i] = 0;
    }
}

void ITG3200::Poll() {
    readI2C(GYRADDR, REGADDR, READ_SIZE, gBuffer);

    gRaw[0] = ((gBuffer[0] << 8) | gBuffer[1]);   // Shift high byte to be high 8 bits and append with low byte.
    gRaw[1] = ((gBuffer[2] << 8) | gBuffer[3]);
    gRaw[2] = ((gBuffer[4] << 8) | gBuffer[5]);

    for (int i=0; i<3; i++) {   // Convert raw values to a nice -1 to 1 range.
    }

    #ifdef DEBUG
        Serial.print("GX: "); Serial.print(gRaw[0]);
        Serial.print("   GY: "); Serial.print(gRaw[1]);
        Serial.print("   GZ: "); Serial.println(gRaw[2]);
    #endif
}

float* ITG3200::Get() {
    return gVal;
}

float ITG3200::Get(int axis) {
    return gVal[axis];
}

