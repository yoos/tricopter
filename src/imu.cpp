/*! \file imu.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Source for IMU class.
 *
 *  I, J, K unit vectors of global coordinate system
 *      I - east
 *      J - north
 *      K - zenith
 *
 *  i, j, k unit vectors of body coordinate system
 *      i - right
 *      j - forward
 *      k - up
 *
 * We keep track of the body frame in relation to the global coordinate frame.
 * That is, orientation is described in the global coordinate system.
 *
 *           [i.I i.J i.K]
 *     DCM = [j.I j.J j.K]
 *           [k.I k.J k.K]
 */

#include "imu.h"

IMU::IMU() {
}

void IMU::init() {
    IMU::reset();
    spln("IMU here!");

    gyro.calibrate(500);

    // Set initial DCM as the identity matrix.
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            gyroDCM[i][j] = (i==j) ? 1.0 : 0.0;

    /*! Calculate DCM offset.
     *
     *  We use an accelerometer to correct for gyro drift. This means that the
     *  IMU will adjust gyroDCM according to whatever the accelerometer thinks
     *  is gravity. However, since it is nearly impossible to mount the
     *  accelerometer perfectly horizontal to the chassis, this also means that
     *  gyroDCM will represent the orientation of the accelerometer, not the
     *  chassis, along the X and Y rotational axes.
     *
     *  The rotational difference between the orientations of the accelerometer
     *  (gyroDCM) and the chassis (bodyDCM) is constant and can be represented
     *  as another rotation matrix we will call offsetDCM. We first calculate a
     *  rotation vector wAOffset based on the accelerometer's reading of
     *  "gravity" when the chassis is horizontal. We then turn wAOffset into
     *  offsetDCM with which we rotate gyroDCM to obtain bodyDCM.
     *
     *  Keep in mind, however, that we still update the IMU based on gyroDCM
     *  and not bodyDCM. This is because it is gyroDCM we are correcting with
     *  the accelerometer's measurement of the gravity vector, and bodyDCM is
     *  only a transformation of that DCM based on offsetDCM. We use bodyDCM
     *  for flight calculations, but gyroDCM is what we keep track of within
     *  the IMU.
     *
     *  As a final note, this is a small-angle approximation! We could do
     *  something fancy with trigonometry, but if the hardware is so poorly
     *  built (or poorly designed) that small-angle approximations become
     *  insufficient, we can't expect software to fix everything.
     */
    #ifdef ACC_WEIGHT
    // k body unit vector in body coordinates.
    kbb[0] = 0.0;
    kbb[1] = 0.0;
    kbb[2] = 1.0;

    wAOffset[0] = ACCEL_X_OFFSET;
    wAOffset[1] = ACCEL_Y_OFFSET;
    wAOffset[2] = ACCEL_Z_OFFSET;
    vCrossP(wAOffset, kbb, wAOffset);

    offsetDCM[0][0] =            1;
    offsetDCM[0][1] =  wAOffset[2];
    offsetDCM[0][2] = -wAOffset[1];
    offsetDCM[1][0] = -wAOffset[2];
    offsetDCM[1][1] =            1;
    offsetDCM[1][2] =  wAOffset[0];
    offsetDCM[2][0] =  wAOffset[1];
    offsetDCM[2][1] = -wAOffset[0];
    offsetDCM[2][2] =            1;

    aVec[0] = 0;
    aVec[1] = 0;
    aVec[2] = -1;
    for (int i=0; i<3; i++) {
        aVecLast[i] = aVec[i];
    }
    //accVar = 100.;
    #endif // ACC_WEIGHT
}

void IMU::update() {
    // ========================================================================
    // Accelerometer
    //     Frame of reference: BODY
    //     Units: G (gravitational acceleration)
    //     Purpose: Measure the acceleration vector aVec with components
    //              codirectional with the i, j, and k vectors. Note that the
    //              gravitational vector is the negative of the K vector.
    // ========================================================================
    #ifdef ACC_WEIGHT
    if (loopCount % ACC_READ_INTERVAL == 0) {
        acc.poll();
        for (int i=0; i<3; i++) {
            aVec[i] = acc.get(i);
        }

        // Take weighted average.
        #ifdef ACC_SELF_WEIGHT
        for (int i=0; i<3; i++) {
            aVec[i] = ACC_SELF_WEIGHT * aVec[i] + (1-ACC_SELF_WEIGHT) * aVecLast[i];
            aVecLast[i] = aVec[i];

            // Kalman filtering?
            //aVecLast[i] = acc.get(i);
            //kalmanUpdate(aVec[i], accVar, aVecLast[i], ACC_UPDATE_SIG);
            //kalmanPredict(aVec[i], accVar, 0.0, ACC_PREDICT_SIG);
        }
        #endif // ACC_SELF_WEIGHT
        accScale = vNorm(aVec);

        // Reduce accelerometer weight if the magnitude of the measured
        // acceleration is significantly greater than or less than 1 g.
        //
        // TODO: Magnitude of acceleration should be reported over telemetry so
        // the "cutoff" value (the constant before the ABS() below) for
        // disregaring acceleration input can be more accurately determined.
        #ifdef ACC_SCALE_WEIGHT
        accScale = (1 - MIN(1, ACC_SCALE_WEIGHT * ABS(accScale - 1)));
        accWeight = ACC_WEIGHT * accScale;
        #else
        accWeight = ACC_WEIGHT;
        #endif // ACC_SCALE_WEIGHT

        // Uncomment the loop below to get accelerometer readings in order to
        // obtain wAOffset.
        //if (loopCount % COMM_LOOP_INTERVAL == 0) {
        //    sp("A(");
        //    sp(aVec[0]*1000); sp(", ");
        //    sp(aVec[1]*1000); sp(", ");
        //    sp(aVec[2]*1000);
        //    sp(")  ");
        //}

        // Express K global unit vector in BODY frame as kgb for use in drift
        // correction (we need K to be described in the BODY frame because
        // gravity is measured by the accelerometer in the BODY frame).
        // Technically we could just create a transpose of gyroDCM, but since
        // we don't (yet) have a magnetometer, we don't need the first two rows
        // of the transpose. This saves a few clock cycles.
        for (int i=0; i<3; i++) {
            kgb[i] = gyroDCM[i][2];
        }

        // Calculate gyro drift correction rotation vector wA, which will be
        // used later to bring KB closer to the gravity vector (i.e., the
        // negative of the K vector). Although we do not explicitly negate the
        // gravity vector, the cross product below produces a rotation vector
        // that we can later add to the angular displacement vector to correct
        // for gyro drift in the X and Y axes.
        vCrossP(kgb, aVec, wA);

        // Divide by ACC_READ_INTERVAL since the acceleration vector is not
        // updated every loop.
        for (int i=0; i<3; i++) {
            wA[i] /= ACC_READ_INTERVAL;
        }
    }
    #endif // ACC_WEIGHT

    // ========================================================================
    // Magnetometer
    //     Frame of reference: BODY
    //     Units: N/A
    //     Purpose: Measure the magnetic north vector mVec with components
    //              codirectional with the body's i, j, and k vectors.
    // ========================================================================
    #ifdef MAG_WEIGHT
    mag.poll();   // ?
    for (int i=0; i<3; i++) {
        mVec[i] = mag.get(i);
    }
    //if (loopCount % COMM_LOOP_INTERVAL == 0) {
    //    sp("M(");
    //    sp(mVec[0]); sp(", ");
    //    sp(mVec[1]); sp(", ");
    //    sp(mVec[2]);
    //    spln(")");
    //}

    // Express J global unit vectory in BODY frame as jgb.
    for (int i=0; i<3; i++) {
        jgb[i] = gyroDCM[i][1];
    }

    // Calculate yaw drift correction vector wM.
    vCrossP(jgb, mVec, wM);
    #endif // MAG_WEIGHT

    // ========================================================================
    // Gyroscope
    //     Frame of reference: BODY
    //     Units: rad/s
    //     Purpose: Measure the rotation rate of the body about the body's i,
    //              j, and k axes.
    // ========================================================================
    gyro.poll();
    for (int i=0; i<3; i++) {
        gVec[i] = gyro.get(i);
    }
    //if (loopCount % COMM_LOOP_INTERVAL == 0) {
    //    sp("G(");
    //    sp(gVec[0]); sp(", ");
    //    sp(gVec[1]); sp(", ");
    //    sp(gVec[2]);
    //    spln(")");
    //}

    // Scale gVec by elapsed time (in seconds) to get angle w*dt in
    // radians, then compute weighted average with the accelerometer and
    // magnetometer correction vectors to obtain final w*dt.
    for (int i=0; i<3; i++) {
        float numerator   = gVec[i] * MASTER_DT/1000000;
        float denominator = 1.0;

        #ifdef ACC_WEIGHT
        if (loopCount % ACC_READ_INTERVAL == 0) {
            numerator   += accWeight * wA[i];
            denominator += accWeight;
        }
        #endif // ACC_WEIGHT

        #ifdef MAG_WEIGHT
        numerator   += MAG_WEIGHT * wM[i];
        denominator += MAG_WEIGHT;
        #endif // MAG_WEIGHT

        wdt[i] = numerator / denominator;
    }

    // ========================================================================
    // Direction Cosine Matrix
    //     Frame of reference: GLOBAL
    //     Units: None (unit vectors)
    //     Purpose: Calculate the components of the body's i, j, and k unit
    //              vectors in the global frame of reference.
    // ========================================================================
    // Skew the rotation vector and sum appropriate components by combining the
    // skew symmetric matrix with the identity matrix. The math can be
    // summarized as follows:
    //
    // All of this is calculated in the BODY frame. If w is the angular
    // velocity vector, let wdt (w*dt) be the angular displacement vector of
    // the DCM over a time interval dt. Let wdt_i, wdt_j, and wdt_k be the
    // components of wdt codirectional with the i, j, and k unit vectors,
    // respectively. Also, let dr be the linear displacement vector of the DCM
    // and dr_i, dr_j, and dr_k once again be the i, j, and k components,
    // respectively.
    //
    // In very small dt, certain vectors approach orthogonality, so we can
    // assume that (draw this out for yourself!):
    //
    //     dr_x = <    0,  dw_k, -dw_j>,
    //     dr_y = <-dw_k,     0,  dw_i>, and
    //     dr_z = < dw_j, -dw_i,     0>,
    //
    // which can be expressed as the rotation matrix:
    //
    //          [     0  dw_k -dw_j ]
    //     dr = [ -dw_k     0  dw_i ]
    //          [  dw_j -dw_i     0 ].
    //
    // This can then be multiplied by the current DCM and added to the current
    // DCM to update the DCM. To minimize the number of calculations performed
    // by the processor, however, we can combine the last two steps by
    // combining dr with the identity matrix to produce:
    //
    //              [     1  dw_k -dw_j ]
    //     dr + I = [ -dw_k     1  dw_i ]
    //              [  dw_j -dw_i     1 ],
    //
    // which we multiply with the current DCM to produce the updated DCM
    // directly.
    //
    // It may be helpful to read the Wikipedia pages on cross products and
    // rotation representation.
    // ========================================================================
    dDCM[0][0] =       1;
    dDCM[0][1] =  wdt[2];
    dDCM[0][2] = -wdt[1];
    dDCM[1][0] = -wdt[2];
    dDCM[1][1] =       1;
    dDCM[1][2] =  wdt[0];
    dDCM[2][0] =  wdt[1];
    dDCM[2][1] = -wdt[0];
    dDCM[2][2] =       1;

    // Multiply the current DCM with the change in DCM and update.
    mProduct(dDCM, gyroDCM, gyroDCM);
    orthonormalize(gyroDCM);

    #ifdef ACC_WEIGHT
    // Rotate gyroDCM with offsetDCM.
    mProduct(offsetDCM, gyroDCM, bodyDCM);
    //orthonormalize(bodyDCM);   // TODO: This shouldn't be necessary.
    #else
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            bodyDCM[i][j] = gyroDCM[i][j];
        }
    }
    #endif // ACC_WEIGHT
}

void IMU::orthonormalize(float inputDCM[3][3]) {
    // Takes 700 us.
    // Orthogonalize the i and j unit vectors (DCMDraft2 Eqn. 19).
    errDCM = vDotP(inputDCM[0], inputDCM[1]);
    vScale(inputDCM[1], -errDCM/2, dDCM[0]);   // i vector correction
    vScale(inputDCM[0], -errDCM/2, dDCM[1]);   // j vector correction
    vAdd(inputDCM[0], dDCM[0], inputDCM[0]);
    vAdd(inputDCM[1], dDCM[1], inputDCM[1]);

    // k = i x j
    vCrossP(inputDCM[0], inputDCM[1], inputDCM[2]);

    // Normalize all three vectors.
    vNorm(inputDCM[0]);
    vNorm(inputDCM[1]);
    vNorm(inputDCM[2]);
}

void IMU::reset() {
}

