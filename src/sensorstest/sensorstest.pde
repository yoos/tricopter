#include "gyro.h"

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    initGyro();
}

void loop()
{
    getGyroscopeData();
    delay(500);
}

