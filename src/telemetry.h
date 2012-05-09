/*! \file telemetry.h
 *  \author Soo-Hyun Yoo
 *  \brief Some functions to make sending telemetry data simpler to code.
 *
 *  Details.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "globals.h"

uint8_t txNewDataIndex = 0;
uint8_t txToSendIndex = 0;
uint8_t txData[SER_WRITE_BUF_LEN];

void queueByte(uint8_t txByte) {
    txData[txNewDataIndex] = txByte;
    txNewDataIndex = (txNewDataIndex + 1) % SER_WRITE_BUF_LEN;
}

void sendTelemetry() {
    int i;
    for (i=0; i<MIN(SER_WRITE_CHUNK_LEN, (SER_WRITE_BUF_LEN + txNewDataIndex - txToSendIndex) % SER_WRITE_BUF_LEN); i++) {
        sw(txData[txToSendIndex]);
        txToSendIndex = (txToSendIndex + 1) % SER_WRITE_BUF_LEN;
    }
}

/*! Report arm status.
 */
void sendArmStatus() {
    char buf[6];
    int len = sprintf(buf, "%d", armCount);

    int i;
    for (i=0; i<len; i++) {
        queueByte(buf[i]);
    }

    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Report target rotation vector (in BODY frame). TODO: targetRot is a
 *  misleading name because this is dwB (delta omega BODY), so to speak (i.e.,
 *  an update to the DCM). There is a target DCM we could calculate but is
 *  unneeded at this moment.
 */
void sendTargetRotation() {
    queueByte(ROT_SER_TAG);
    for (int i=0; i<3; i++) {
        queueByte((uint8_t) ((targetAngPos[i]+PI)*250/(2*PI)));
    }
    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Report motor values.
 */
void sendMotorValues() {
    queueByte(MOT_SER_TAG);
    for (int i=0; i<4; i++) {
        queueByte((uint8_t) (pwmOut[i]-TMIN)*250/376);
    }
    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Datafeed to serialmon.py for visualization.
 */
void sendDCM() {
    queueByte(DCM_SER_TAG);   // Index tag 'DCM'.
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            queueByte((uint8_t) ((int8_t) (bodyDCM[i][j] * 125) + 125));
        }
    }
    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Report PID gains and values.
 */
void sendPIDData() {
    queueByte(PID_SER_TAG);   // Index tag 'PID'.
    queueByte((uint8_t) PID[PID_ANG_POS_X].P);
    queueByte((uint8_t) PID[PID_ANG_RATE_X].P);
    queueByte((uint8_t) (-100*PID[PID_ANG_RATE_X].D));
    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Signal end of telemetry.
 */
void sendTelemetryEnd(int nextRuntime) {
    // Report loop time.
    char buf[10];
    int len = sprintf(buf, "%d", (unsigned int) (micros() - (nextRuntime - MASTER_DT)));

    int i;
    for (i=0; i<len; i++) {
        queueByte(buf[i]);
    }

    queueByte(0xde); queueByte(0xad); queueByte(0xbe); queueByte(0xef);
}

#endif // TELEMETRY_H

