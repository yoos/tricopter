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
}

void IMU::Update() {
    myGyr.Poll();
    myAcc.Poll();

    adcAvg[0] = myAcc.Get(0);
    adcAvg[1] = myAcc.Get(1);
    adcAvg[2] = myAcc.Get(2);
    adcAvg[3] = myGyr.GetRate(0);
    adcAvg[4] = myGyr.GetRate(1);
    adcAvg[5] = myGyr.GetRate(2);


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

void IMU::Get() {
    angle = KH * (angle + myGyr.GetAngle(1)*DT) + (1-KH) * myAcc.Get(0); // Doesn't work, but a start to complementary filtering?
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

