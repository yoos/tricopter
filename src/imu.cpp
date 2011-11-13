// ============================================================================
// imu.cpp
// ============================================================================
// I, J, K unity vectors of global coordinate system
//     I - east
//     J - north
//     K - zenith
// i, j, k unity vectors of body coordinate system
//     i - right
//     j - forward
//     k - up
// ============================================================================
//       [I.i , I.j, I.k]
// DCM = [J.i , J.j, J.k]
//       [K.i , K.j, K.k]
// ============================================================================

#include "imu.h"

IMU::IMU() : myAcc(4, 2),   // range, bandwidth: DS p. 27
             myGyr(3)   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s
{}

void IMU::Init() {
    IMU::Reset();
    spln("IMU here!");

    // Calibrate sensors. TODO: use accelerometer to find initial tricopter
    // orientation.
    myGyr.Calibrate(500);

    // Set initial DCM as the identity matrix.
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            currentDCM[i][j] = (i==j) ? 1.0 : 0.0;
    currentAngle[PITCH] = 0;
    currentAngle[ROLL] = 0;
}

void IMU::Update() {
    // ========================================================================
    // Acelerometer
    //     Frame of reference: BODY
    //     Units: G (gravitational acceleration)
    //     Purpose: Measure the acceleration vector aVec with components
    //              codirectional with the i, j, and k vectors. Note that the
    //              gravitational vector is the negative of the K vector.
    // ========================================================================
    myAcc.Poll();
    aVec[0] = myAcc.Get(0);
    aVec[1] = myAcc.Get(1);
    aVec[2] = myAcc.Get(2);
    vNorm(aVec);

    // TODO: FIX THIS: Calculate correction vector to bring currentDCM's K
    // vector closer to aVec vector (K vector according to accelerometer)
    vCrossP(aVec, currentDCM[2], wA);

    // ========================================================================
    // Gyroscope
    //     Frame of reference: BODY
    //     Units: rad/s
    //     Purpose: Measure the rotation rate of the body about the body's X,
    //              Y, and Z axes (i.e., i, j, k).
    // ========================================================================
    myGyr.Poll();
    gVec[0] = myGyr.GetRate(0);
    gVec[1] = myGyr.GetRate(1);
    gVec[2] = myGyr.GetRate(2);

    // Scale gVec by elapsed time (in seconds) to get angle w*dt in radians,
    // then compute weighted average with the accelerometer correction vector.
    for (int i=0; i<3; i++) {
        wdt[i] = gVec[i] * SYSINTRV/1000;
        wdt[i] = (wdt[i] + ACC_WEIGHT*wA[i]) / (1.0 + ACC_WEIGHT);
    }

    // ========================================================================
    // Direction Cosine Matrix
    //     Frame of reference: GLOBAL
    //     Units: None (unit vectors)
    //     Purpose: Calculate the components of the body's i, j, and k unity
    //              vectors in the global frame of reference.
    // ========================================================================

    // Skew the rotation vector and add to appropriate axis by combining the
    // skew symmetric matrix with the identity matrix.
    vSkewPlusIdentity(wdt, dDCM);

    // Multiply the current DCM with the change in DCM and update.
    mProduct(dDCM, currentDCM, currentDCM);

    // Datafeed to serialmon.py for visualization.
    if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
        sp("DCM ");   // Index tag 'DCM'.
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                sp(currentDCM[i][j]);
                sp(" ");
            }
        }
    }

    // Orthonormalize the DCM (DCMDraft2 Eqn. 19).
    vDotP(currentDCM[0], currentDCM[1], errDCM);
    vScale(currentDCM[1], -errDCM/2, dDCM[0]);   // i vector correction
    vScale(currentDCM[0], -errDCM/2, dDCM[1]);   // j vector correction
    vAdd(currentDCM[0], dDCM[0], currentDCM[0]);
    vAdd(currentDCM[1], dDCM[1], currentDCM[1]);

    // Z = X x Y
    vCrossP(currentDCM[0], currentDCM[1], currentDCM[2]);
    vNorm(currentDCM[0]);
    vNorm(currentDCM[1]);
    vNorm(currentDCM[2]);
}

void IMU::Reset() {
}

