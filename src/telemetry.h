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

/*! Report trim values.
 */
void sendTrimValues() {
    queueByte(TRIM_SER_TAG);
    for (int i=0; i<2; i++) {
        queueByte((uint8_t) (trimAngle[i]*20/PI * 125) + 125);   // Hopefully the magnitude of wAOffset will remain under PI/20.
    }
    queueByte(FIELD_SER_TAG); queueByte(FIELD_SER_TAG);
}

/*! Report motor values.
 */
void sendMotorValues() {
    queueByte(MOT_SER_TAG);
    queueByte((uint8_t) ((pwmOut[MOTOR_T]-TMIN)*250/329));   // 761 - 432 = 329.
    queueByte((uint8_t) ((pwmOut[MOTOR_R]-TMIN)*250/329));   // 761 - 432 = 329.
    queueByte((uint8_t) ((pwmOut[MOTOR_L]-TMIN)*250/329));   // 761 - 432 = 329.
    queueByte((uint8_t) ((pwmOut[SERVO_T]-SERVO_MIN) * 250 / (SERVO_MAX - SERVO_MIN)));
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
    queueByte((uint8_t) ((int) (  10*PID[PID_ANG_POS_X].P) % 250));
    queueByte((uint8_t) ((int) (  10*PID[PID_ANG_VEL_X].P) % 250));
    queueByte((uint8_t) ((int) (-100*PID[PID_ANG_VEL_X].D) % 250));
    queueByte((uint8_t) ((int) (  10*PID[PID_ANG_POS_Z].P) % 250));
    queueByte((uint8_t) ((int) (   1*PID[PID_ANG_VEL_Z].P) % 250));
    queueByte((uint8_t) ((int) (-100*PID[PID_ANG_VEL_Z].D) % 250));
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

