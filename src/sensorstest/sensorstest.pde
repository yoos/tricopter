#include "gyro.h"
#include "accelerometer.h"

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    initGyro();
    initAccel();
}

void loop()
{
    readGyro();
    readAccel();
    delay(500);
}

