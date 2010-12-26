#include "imu.h"

IMU::IMU() : myAcc(4, 2),   // range, bandwidth: DS p. 27
             myGyr(2)   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s
{
    IMU::reset();
    #ifdef DEBUG
    Serial.println("IMU here!");
    #endif
}

void IMU::Update() {
    myAcc.Poll();
    myGyr.Poll();
    #ifdef DEBUG
    Serial.println("IMU updated.");
    #endif
}

void IMU::deadReckoning() {
    // Update position and orientation regularly
    if (millis() - lastTime > IMU_SAMPLE_INTERVAL) {
        for (int i; i<3; i++) {
            curRot[i] = curRot[i] + myGyr.Get(i) * (IMU_SAMPLE_INTERVAL/1000);
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

