#include "bma180.h"

BMA180::BMA180(uint8_t range, uint8_t bw) {
    readI2C(DEVICE, 0x00, 1, aBuffer);
    
    #ifdef DEBUG
        Serial.print("Accelerometer ID = ");
        Serial.println((int) aBuffer[0]);
    #endif
    
    if (aBuffer[0] == 3) {
        Serial.println("BMA180 successfully initialized!");
    }

    // Set ee_w bit
    readI2C(DEVICE, CTRLREG0, 1, aBuffer);
    aBuffer[0] |= 0x10;   // Bitwise OR operator to set ee_w bit.
    sendI2C(DEVICE, CTRLREG0, aBuffer[0]);   // Have to set ee_w to write any other registers.

    // Set BandWidth
    readI2C(DEVICE, BWTCS, 1, aBuffer);
    aBuffer[1] = bw;
    aBuffer[1] = aBuffer[1] << 4;
    aBuffer[0] &= (~BWMASK);
    aBuffer[0] |= aBuffer[1];
    sendI2C(DEVICE, BWTCS, aBuffer[0]);   // Keep tcs<3:0> in BWTCS, but write new BW.

    // Set Range
    readI2C(DEVICE, OLSB1, 1, aBuffer);
    aBuffer[1] = range;
    aBuffer[1] = (aBuffer[1] << RANGESHIFT);
    aBuffer[0] &= (~RANGEMASK);
    aBuffer[0] |= aBuffer[1];
    sendI2C(DEVICE, OLSB1, aBuffer[0]);   // Write new range data, keep other bits the same.

//  if (aBuffer[0] == 3) {
//      eCode = sendI2C(DEVICE, 0x0D, B0001);
//      eCode = sendI2C(DEVICE, 0x20, B00001000);
//      eCode = sendI2C(DEVICE, 0x35, B0100);
//  }

    // Zero buffer.
    for (int i; i<READ_SIZE; i++) {
        aBuffer[i] = 0;
    }
}

void BMA180::Poll() {
    eCode = readI2C(DEVICE, REG_ADDR, READ_SIZE, aBuffer);   // Read acceleration data
    aVal[0] = (aBuffer[1] << 8) | (aBuffer[0] >> 2);
    aVal[1] = (aBuffer[3] << 8) | (aBuffer[2] >> 2);
    aVal[2] = (aBuffer[5] << 8) | (aBuffer[4] >> 2);

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


