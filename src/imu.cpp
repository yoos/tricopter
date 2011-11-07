#include "imu.h"

IMU::IMU() : myAcc(4, 2),   // range, bandwidth: DS p. 27
             myGyr(3)   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s
{}

void IMU::Init() {
    // aVec = {0, 0, 0};
    // gVec = {0, 0, 0};
    // oVec = {0, 0, 0};
    // oVecP = {0, 0, 0};
    // oVecI = {0, 0, 0};
    // tmpVec = {0, 0, 0};

    // DCM = {{1, 0, 0},
    //        {0, 1, 0},
    //        {0, 0, 1}};
    // tmpMat = {{0, 0, 0},
    //           {0, 0, 0},
    //           {0, 0, 0}};
    IMU::Reset();
    #ifdef DEBUG
    spln("IMU here!");
    #endif

/* Calibrate sensors if needed and find initial tricopter orientation. */
    myGyr.Calibrate(100);

    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            currentDCM[i][j] = (i==j) ? 1.0 : 0.0;
    currentAngle[PITCH] = 0;
    currentAngle[ROLL] = 0;
}

void IMU::Update() {
    myGyr.Poll();
    myAcc.Poll();

    for (int i=0; i<3; i++) {
        aVec[i] = myAcc.Get(i);
        gVec[i] = myGyr.GetRate(i);
    }

    // XXX: just while I test...
    currentAngle[PITCH] = atan2(aVec[1], -aVec[2]);   // Pitch
    currentAngle[ROLL]  = atan2(aVec[0], -aVec[2]);   // Roll
    sp("[");
    sp(currentAngle[PITCH]);
    sp(" ");
    sp(currentAngle[ROLL]);
    sp("] ");
    
    #ifdef DEBUG
    spln("IMU updated.");
    #endif


    // XXX Following code from PICQ
    
    //---------------
    // I,J,K unity vectors of global coordinate system I-North,J-West,K-zenith
    // i,j,k unity vectors of body's coordiante system  i-"nose", j-"left wing", k-"top"
    //---------------
    //            [I.i , I.j, I.k]
    // DCM =      [J.i , J.j, J.k]
    //            [K.i , K.j, K.k]  

    //---------------
    //Acelerometer
    //---------------
    //Accelerometer measures gravity vector G in body coordinate system
    //Gravity vector is the reverse of K unity vector of global system expressed in local coordinates
    //K vector coincides with the z coordinate of body's i,j,k vectors expressed in global coordinates (K.i , K.j, K.k)
    //Acc can estimate global K vector(zenith) measured in body's coordinate systems (the reverse of gravitation vector)
    Kacc[0] = -aVec[0];
    Kacc[1] = -aVec[1];
    Kacc[2] = -aVec[2];
    vNorm(Kacc);
    //calculate correction vector to bring currentDCM's K vector closer to Acc vector (K vector according to accelerometer)
    vCrossP(currentDCM[2], Kacc, wA);    // wA = Kgyro x     Kacc , rotation needed to bring Kacc to Kgyro

    //---------------
    //Magnetomer
    //---------------
    //calculate correction vector to bring currentDCM's I vector closer to Mag vector (I vector according to magnetometer)
    //in the absense of magnetometer let's assume North vector (I) is always in XZ plane of the device (y coordinate is 0)
    Imag[0] = sqrt(1-currentDCM[0][2]*currentDCM[0][2]);
    Imag[1] = 0;
    Imag[2] = currentDCM[0][2];
    
    vCrossP(currentDCM[0], Imag, wM);    // wM = Igyro x Imag, roation needed to bring Imag to Igyro

    //---------------
    //currentDCM
    //---------------
    w[0] = gVec[0];   //rotation rate about accelerometer's X axis (GY output) in rad/ms
    w[1] = gVec[1];   //rotation rate about accelerometer's Y axis (GX output) in rad/ms
    w[2] = gVec[2];   //rotation rate about accelerometer's Z axis (GZ output) in rad/ms
    for (int i=0; i<3; i++) {
        w[i] *= SYSINTRV/1000;   //scale by elapsed time (in s) to get angle in radians
        //compute weighted average with the accelerometer correction vector
        w[i] = (w[i] + ACC_WEIGHT*wA[i] + MAG_WEIGHT*wM[i])/(1.0+ACC_WEIGHT+MAG_WEIGHT);
    }
    //sp("(");
    //sp(w[0]*1000);
    //sp(" ");
    //sp(w[1]*1000);
    //sp(" ");
    //sp(w[2]*1000);
    //sp(") ");
    
    imu_dcm_rotate(currentDCM, w);

    //for (int i=2; i<3; i++) {
    //    sp("(");
    //    sp(currentDCM[i][0]*1000);
    //    sp(" ");
    //    sp(currentDCM[i][1]*1000);
    //    sp(" ");
    //    sp(currentDCM[i][2]*1000);
    //    sp(") ");
    //}
}

//void IMU::deadReckoning() {
//    // Update position and orientation regularly
//    if (millis() - lastTime > IMU_SAMPLE_INTERVAL) {
//        for (int i; i<3; i++) {
//            curRot[i] = curRot[i] + myGyr.GetRate(i) * (IMU_SAMPLE_INTERVAL/1000);
//        }
//    }
//
//    // Update X position
////  curPos[0] = accel.getX()*sec(gyro.getY         );
//
//}

void IMU::Reset() {
    for (int i; i<3; i++) {
        curRot[i] = 0;
        curPos[i] = 0;
    }
}


// XXX From PICQ: Bring DCM matrix in order - adjust values to make orthonormal (or at least closer to orthonormal)
// Note: dcm and dcmResult can be the same.
void imu_dcm_orthonormalize(float dcm[3][3]) {
    //err = X . Y ,  X = X - err/2 * Y , Y = Y - err/2 * X  (DCMDraft2 Eqn.19)
    float err;
    vDotP((float*)(dcm[0]), (float*)(dcm[1]), err);
    float delta[2][3];
    vScale((float*)(dcm[1]), -err/2, (float*)(delta[0]));
    vScale((float*)(dcm[0]), -err/2, (float*)(delta[1]));
    vAdd((float*)(dcm[0]), (float*)(delta[0]), (float*)(dcm[0]));
    vAdd((float*)(dcm[1]), (float*)(delta[0]), (float*)(dcm[1]));

    //Z = X x Y  (DCMDraft2 Eqn. 20) , 
    vCrossP((float*)(dcm[0]), (float*)(dcm[1]), (float*)(dcm[2]));
    //re-nomralization
    vNorm((float*)(dcm[0]));
    vNorm((float*)(dcm[1]));
    vNorm((float*)(dcm[2]));
}

// XXX From PICQ: rotate DCM matrix by a small rotation given by angular rotation vector w
// See http://gentlenav.googlecode.com/files/DCMDraft2.pdf
void imu_dcm_rotate(float dcm[3][3], float w[3]) {
    //float W[3][3];    
    //creates equivalent skew symetric matrix plus identity matrix
    //vector3d_skew_plus_identity((float*)w,(float*)W);
    //float dcmTmp[3][3];
    //matrix_multiply(3,3,3,(float*)W,(float*)dcm,(float*)dcmTmp);
    
    int i;
    float dR[3];
    //update matrix using formula R(t+1)= R(t) + dR(t) = R(t) + w x R(t)
    for(i=0;i<3;i++){
        vCrossP(w, dcm[i], dR);
        vAdd(dcm[i], dR, dcm[i]);
    }        

    //make matrix orthonormal again
    imu_dcm_orthonormalize(dcm);
}

