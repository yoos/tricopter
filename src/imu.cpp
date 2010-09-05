#include "imu.h"

IMU::IMU(Gyro& g, Accelerometer& a) {
    gyro = g;
    accel = a;
    IMU::reset();
}

void IMU::deadReckoning() {
    // Update position and orientation regularly
    if (millis() - lastTime > IMU_SAMPLE_INTERVAL) {
        for (int i; i<3; i++) {
            curRot[i] = curRot[i] + gyro.get(i) * (IMU_SAMPLE_INTERVAL/1000);
        }
    }

    // Update X position
    curPos[0] = accel.getX()*sec(gyro.getY         );

}

void IMU::reset() {
    for (int i; i<3; i++) {
        curRot[i] = 0;
        curPos[i] = 0;
    }
}

