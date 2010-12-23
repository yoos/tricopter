#include "itg3200.h"

ITG3200::ITG3200(uint8_t range) {
    readI2C(GYRADDR, 0x00, 1, gBuffer);   // Who am I?
    
    #ifdef DEBUG
        Serial.print("ITG-3200 ID = ");
        Serial.println((int) gBuffer[0]);
    #endif

    // Configure ITG-3200
    // Refer to DS Section 8: Register Description.
    sendI2C(GYRADDR, 0x15, 0x18);   // 00011000 -- Sample rate divider is 24(+1), so 40 Hz

    // Set Range
    readI2C(GYRADDR, 0x16, 1, gBuffer);
    gBuffer[1] = range;
    gBuffer[1] = (gBuffer[1] << 3);   // See DS.
    gBuffer[0] |= gBuffer[1];
    sendI2C(GYRADDR, 0x16, gBuffer[0]);

    #ifdef DEBUG
        Serial.println("ITG-3200 configured!");
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
        float tmp;
        if (gRaw[i] >= 0x8000)   // If zero to negative rot. vel.: 2^16-1 to 2^15...
            tmp = -((signed) (0xFFFF - gRaw[i]));   // ...subtract from 2^16-1.
        gVal[i] = tmp/0x8000;   // Divide by maximum magnitude.
    }

    #ifdef DEBUG
        Serial.print("GX: "); Serial.print(gVal[0]);
        Serial.print("   GY: "); Serial.print(gVal[1]);
        Serial.print("   GZ: "); Serial.println(gVal[2]);
    #endif
}

float* ITG3200::Get() {
    return gVal;
}

float ITG3200::Get(int axis) {
    return gVal[axis];
}

