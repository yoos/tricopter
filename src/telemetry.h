/*! \file telemetry.h
 *  \author Soo-Hyun Yoo
 *  \brief Some functions to make sending telemetry data simpler to code.
 *
 *  Details.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "globals.h"

void sendTelemetry(int nextRuntime) {
    // ========================================================================
    // Report arm status.
    // ========================================================================
    #ifdef SEND_ARM_STATUS
    sw(armCount);
    sw(FIELD_SER_TAG); sw(FIELD_SER_TAG);
    #endif // SEND_ARM_STATUS

    // ========================================================================
    // Report target rotation vector (in BODY frame). TODO: targetRot is a
    // misleading name because this is dwB (delta omega BODY), so to speak
    // (i.e., an update to the DCM). There is a target DCM we could calculate
    // but is unneeded at this moment.
    // ========================================================================
    #ifdef SEND_TARGET_ROTATION
    sw(ROT_SER_TAG);
    for (int i=0; i<3; i++) {
        sw((250*(targetRot[i]+PI)/(2*PI)+1));
    }
    sw(FIELD_SER_TAG); sw(FIELD_SER_TAG);
    #endif // SEND_TARGET_ROTATION

    // ========================================================================
    // Report motor values.
    // ========================================================================
    #ifdef SEND_MOTOR_VALUES
    sw(MOT_SER_TAG);
    for (int i=0; i<4; i++) {
        sw((byte) ((int) (pwmOut[i]-750.0)*250.0/1450.0));
        //sw((byte*) &pwmOut[i], 4);
    }
    sw(FIELD_SER_TAG); sw(FIELD_SER_TAG);
    #endif // SEND_MOTOR_VALUES

    // ========================================================================
    // Datafeed to serialmon.py for visualization.
    // ========================================================================
    #ifdef SEND_DCM
    sw(DCM_SER_TAG);   // Index tag 'DCM'.
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            //sw((byte*) &gyroDCM[i][j], 4);
            sw((byte) (250*(gyroDCM[i][j]+1)/2+1));
        }
    }
    sw(FIELD_SER_TAG); sw(FIELD_SER_TAG);
    #endif // SEND_DCM

    // ========================================================================
    // Report PID gains and values.
    // ========================================================================
    #ifdef SEND_PID
    sw(PID_SER_TAG);   // Index tag 'PID'.
    sw((byte) ((int) PID[PID_ROT_X].P));
    sw((byte) ((int) PID[PID_ROT_X].I));
    sw((byte) ((int) -PID[PID_ROT_X].D));
    sw(FIELD_SER_TAG); sw(FIELD_SER_TAG);
    #endif // SEND_PID

    // Report loop time.
    sp((int) (micros() - (nextRuntime - MASTER_DT)));

    sw(0xde); sw(0xad); sw(0xbe); sw(0xef);
}

#endif // TELEMETRY_H
