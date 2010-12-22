#include "bma180.h"

BMA180::BMA180(uint8_t range, uint8_t bw) {
    readI2C(ACCADDR, 0x00, 1, aBuffer);
    
    #ifdef DEBUG
        Serial.print("Accelerometer ID = ");
        Serial.println((int) aBuffer[0]);
    #endif
    
    if (aBuffer[0] == 3) {
        Serial.println("BMA180 successfully initialized!");
    }

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

//  if (aBuffer[0] == 3) {
//      eCode = sendI2C(ACCADDR, 0x0D, B0001);
//      eCode = sendI2C(ACCADDR, 0x20, B00001000);
//      eCode = sendI2C(ACCADDR, 0x35, B0100);
//  }

    // Zero buffer.
    for (int i; i<READ_SIZE; i++) {
        aBuffer[i] = 0;
    }
}

void BMA180::Poll() {
    readI2C(ACCADDR, REGADDR, READ_SIZE, aBuffer);   // Read acceleration data
    aVal[0] = (((uint16_t) (aBuffer[1] << 6)) | ((uint16_t) (aBuffer[0] >> 2)));
    aVal[1] = (((uint16_t) (aBuffer[3] << 6)) | ((uint16_t) (aBuffer[2] >> 2)));
    aVal[2] = (((uint16_t) (aBuffer[5] << 6)) | ((uint16_t) (aBuffer[4] >> 2)));
    
    #ifdef DEBUG
        sprintf(accelStr, "AX: %6u AY: %6u AZ: %6u", aVal[0], aVal[1], aVal[2]);   // Interpret aVal as unsigned int.
        Serial.print(accelStr);
        Serial.print(10, BYTE);
    #endif
}

uint16_t* BMA180::Get() {
    return aVal;
}

uint16_t BMA180::Get(int axis) {
    return aVal[axis];
}


