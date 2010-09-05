#include "control.h"
#include "watchdog.h"
#include "gyro.h"
#include "accel.h"
#include "imu.h"
#include "pilot.h"
#include "globals.h"


int flightMode = OFF;


void setup() {

    Gyro gyro(GYRO_X, GYRO_Y, GYRO_Z);
    Accelerometer accel(ACCEL_X, ACCEL_Y, ACCEL_Z);
    IMU imu(gyro, accel);
    Pilot pilot(MOTOR_L, MOTOR_R, MOTOR_T);


}


void loop() {






}

// Need two watchdogs: one for RC connection that when not fed, makes tricopter switch to autonomous mode, and one for onboard code, which when not fed kills bot.
