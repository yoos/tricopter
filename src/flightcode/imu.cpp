#include "imu.h"

IMU::IMU() : myAcc(4, 2),   // range, bandwidth: DS p. 27
             myGyr(3)   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s
{
    IMU::reset();
    #ifdef DEBUG
    Serial.println("IMU here!");
    #endif
}

void IMU::Update() {
    myGyr.Poll();
    myAcc.Poll();
    #ifdef DEBUG
    Serial.println("IMU updated.");
    #endif

    Serial.print("Acc: ");
    Serial.print(myAcc.Get(AX)); Serial.print("  ");
    Serial.print(myAcc.Get(AY)); Serial.print("  ");
    Serial.print(myAcc.Get(AZ)); Serial.print("    ");
    Serial.print("Gyr: ");
    Serial.print(myGyr.GetAngle(GX)); Serial.print("  ");
    Serial.print(myGyr.GetAngle(GY)); Serial.print("  ");
    Serial.print(myGyr.GetAngle(GZ)); Serial.print("    ");
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

