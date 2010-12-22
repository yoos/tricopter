#include "itg3200.cpp"
#include "bma180.cpp"

int main()
{
    Serial.begin(9600);
    Wire.begin();
    initGyro();
    BMA180 myAccel;

    while (true) {
        readGyro();
        myAccel.Poll();
        delay(500);
    }
}

