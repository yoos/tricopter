#include "gyro.h"
#include "accelerometer.h"

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    initGyro();
    initAccelerometer();
}

void loop()
{
    getGyroscopeData();
    getAccelerometerData();
    delay(500);
}

