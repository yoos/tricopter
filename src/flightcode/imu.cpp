#include "imu.h"

IMU::IMU() : myAcc(4, 2),   // range, bandwidth: DS p. 27
             myGyr(3)   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s
{
    IMU::reset();
    #ifdef DEBUG
    Serial.println("IMU here!");
    #endif

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
}

void IMU::Init() {
/* Calibrate sensors if needed and find initial tricopter orientation. */
    myGyr.Calibrate(100);
    imu_init();
}

void IMU::Update() {
    myGyr.Poll();
    myAcc.Poll();

    for (int i=0; i<3; i++) {
        aVec[i] = myAcc.Get(i);
        gVec[i] = myGyr.GetRate(i);
    }
    
    imu_update();

    #ifdef DEBUG
    Serial.println("IMU updated.");
    #endif

    // Serial.print("A: ");
    // Serial.print(myAcc.Get(AX)); Serial.print(" ");
    // Serial.print(myAcc.Get(AY)); Serial.print(" ");
    // Serial.print(myAcc.Get(AZ)); Serial.print("  ");
    // Serial.print("G: ");
    // Serial.print(myGyr.GetAngle(GX)); Serial.print(" ");
    // Serial.print(myGyr.GetAngle(GY)); Serial.print(" ");
    // Serial.print(myGyr.GetAngle(GZ)); Serial.print("  ");
}

float* IMU::GetDCM() {
    return dcmGyro;
}

void IMU::deadReckoning() {
    // Update position and orientation regularly
    if (millis() - lastTime > IMU_SAMPLE_INTERVAL) {
        for (int i; i<3; i++) {
            curRot[i] = curRot[i] + myGyr.GetRate(i) * (IMU_SAMPLE_INTERVAL/1000);
        }
    }

    // Update X position
//  curPos[0] = accel.getX()*sec(gyro.getY         );

}

void IMU::reset() {
    for (int i; i<3; i++) {
        curRot[i] = 0;
        curPos[i] = 0;
    }
}

