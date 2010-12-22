#include "bma180.h"

BMA180::BMA180() {
    eCode = readI2C(DEVICE, 0x00, 1, aBuffer);
    Serial.print("Accelerometer ID = ");
    Serial.println(aBuffer[0]);

    if (aBuffer[0] == 3) {
        eCode = sendI2C(DEVICE, 0x0D, B0001);
        eCode = sendI2C(DEVICE, 0x20, B00001000);
        eCode = sendI2C(DEVICE, 0x35, B0100);
    }

    if (eCode == 0) {
        Serial.println("BMA180 successfully initialized!");
    }

    for (int i; i<READ_SIZE; i++) {
        aBuffer[i] = 0;
    }
}

void BMA180::Poll() {
    eCode = readI2C(DEVICE, REG_ADDR, READ_SIZE, aBuffer);   // Read acceleration data
    aVal[0] = ((aBuffer[1]) << 8) | aBuffer[0];
    aVal[1] = ((aBuffer[3]) << 8) | aBuffer[2];
    aVal[2] = ((aBuffer[5]) << 8) | aBuffer[4];

    #ifdef DEBUG
        sprintf(accelStr, "AX: %d   AY: %d   AZ: %d", aVal[0], aVal[1], aVal[2]);
        Serial.print(accelStr);
        Serial.print(10, BYTE);
    #endif
}

double* BMA180::Get() {
    return aVal;
}

double BMA180::Get(int axis) {
    return aVal[axis];
}


