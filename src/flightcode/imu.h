#ifndef IMU_H
#define IMU_H

#include "itg3200.cpp"
#include "bma180.cpp"
#include "triMath.h"

// Axis numbers
#define AX 0
#define AY 1
#define AZ 2
#define GX 0
#define GY 1
#define GZ 2

#define ACC_WEIGHT_MAX 0.02            //maximum accelerometer weight in accelerometer-gyro fusion formula
                                    //this value is tuned-up experimentally: if you get too much noise - decrease it
                                    //if you get a delayed response of the filtered values - increase it
                                    //starting with a value of  0.01 .. 0.05 will work for most sensors

#define ACC_ERR_MAX  0.3            //maximum allowable error(external acceleration) where accWeight becomes 0


class IMU {
    BMA180 myAcc;
    ITG3200 myGyr;

    float aVec[3];   // Acceleration vector (uncorrected outputs).
    float gVec[3];   // Gyro vector (uncorrected outputs).
    // float oVec[3];   // Omega vector (calculated rotation rate vector; see p. 13 of DCM draft 2)
    // float oVecP[3];   // Omega vector P
    // float oVecI[3];   // Omega vector I
    // float tmpVec[3];   // Temporary vector for calculations.
    
    // float adcAvg[6];   // ADC outputs of 3-axis accel/gyro. Needed by picquadcontroller imu.h.

    float angle;

    int lastTime;
    float curPos[3]; //   Array of X, Y, and Z coordinates relative to start position
    float curRot[3]; //   Array of X, Y, and Z rotational angles relative to start orientation

    // float DCM[3][3];   // Direction cosine matrix.
    // float dMat[3][3];   // System update matrix.
    // float tmpMat[3][3];   // Temporary matrix.




    float dcmAcc[3][3];                //dcm matrix according to accelerometer
    float dcmGyro[3][3];            //dcm matrix according to gyroscopes
    float dcmEst[3][3];                //estimated dcm matrix by fusion of accelerometer and gyro
    
    //bring dcm matrix in order - adjust values to make orthonormal (or at least closer to orthonormal)
    //Note: dcm and dcmResult can be the same
    void dcm_orthonormalize(float dcm[3][3]){
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
    
    
    //rotate DCM matrix by a small rotation given by angular rotation vector w
    //see http://gentlenav.googlecode.com/files/DCMDraft2.pdf
    void dcm_rotate(float dcm[3][3], float w[3]){
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
        dcm_orthonormalize(dcm);
    }
    
    
    //-------------------------------------------------------------------
    // imu_init
    //-------------------------------------------------------------------
    void imu_init() {
        unsigned char i,j;
        for (i=0; i<3; i++)
            for (j=0; j<3; j++)
                dcmGyro[i][j] = (i==j) ? 1.0 : 0.0;
    }
    
    
    
    //-------------------------------------------------------------------
    // imu_update
    //-------------------------------------------------------------------
    #define ACC_WEIGHT 0.01        //accelerometer data weight relative to gyro's weight of 1
    #define MAG_WEIGHT 0.0        //magnetometer data weight relative to gyro's weight of 1
    
    void imu_update() {
        int i;
    
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
        float Kacc[3];            
        //Acc can estimate global K vector(zenith) measured in body's coordinate systems (the reverse of gravitation vector)
        Kacc[0] = aVec[0]; // -getAcclOutput(0);    
        Kacc[1] = aVec[1]; // -getAcclOutput(1);
        Kacc[2] = aVec[2]; // -getAcclOutput(2);
        vNorm(Kacc);
        //calculate correction vector to bring dcmGyro's K vector closer to Acc vector (K vector according to accelerometer)
        float wA[3]; 
        vCrossP(dcmGyro[2], Kacc, wA);    // wA = Kgyro x     Kacc , rotation needed to bring Kacc to Kgyro
    
        //---------------
        //Magnetomer
        //---------------
        //calculate correction vector to bring dcmGyro's I vector closer to Mag vector (I vector according to magnetometer)
        float Imag[3];
        float wM[3];
        //in the absense of magnetometer let's assume North vector (I) is always in XZ plane of the device (y coordinate is 0)
        Imag[0] = sqrt(1-dcmGyro[0][2]*dcmGyro[0][2]);
        Imag[1] = 0;
        Imag[2] = dcmGyro[0][2];
        
        vCrossP(dcmGyro[0], Imag, wM);    // wM = Igyro x Imag, roation needed to bring Imag to Igyro
    
        //---------------
        //dcmGyro
        //---------------
        float w[3];                    //gyro rates (angular velocity of a global vector in local coordinates)
        w[0] = -gVec[1];    //rotation rate about accelerometer's X axis (GY output) in rad/ms
        w[1] = -gVec[0];    //rotation rate about accelerometer's Y axis (GX output) in rad/ms
        w[2] = -gVec[2];    //rotation rate about accelerometer's Z axis (GZ output) in rad/ms
        for (i=0; i<3; i++) {
            w[i] *= SYSINTRV;                //scale by elapsed time to get angle in radians
            //compute weighted average with the accelerometer correction vector
            w[i] = (w[i] + ACC_WEIGHT*wA[i] + MAG_WEIGHT*wM[i])/(1.0+ACC_WEIGHT+MAG_WEIGHT);
        }
        
        dcm_rotate(dcmGyro, w);
    
        //Output for PicQuadController_GYRO_DEBUG1.scc
        //only output data ocasionally to allow computer to process data
        /*
        if(0 == imu_sequence % 4){
            printf("%.5f,",(double)interval_ms);
            print_float_list(3,(float*)w);
            printf(",%.2f,%.2f,%.2f",adcAvg[3+1],adcAvg[3+0],adcAvg[3+2]);
    
            printf("\n ");
        }
        */
    
        
        //Output for: PICQUADCONTROLLER_DEBUG1.pde
        //only output data ocasionally to allow computer to process data
        // if(0 == imu_sequence % 16){
        //     printf("%.2f,",(double)SYSINTRV);
        //     print_float_list(3,(float*)Kacc);
        //     printf(", ");
        //     print_float_list(9,(float*)dcmGyro);
        //     printf("\n");
        // }
        
    }


public:
    IMU();
    void Init();
    void Update();
    void GetDCM();
    void deadReckoning();
    void reset();
};



#endif

