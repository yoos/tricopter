#include "itg3200.cpp"
#include "bma180.cpp"

char genStr[512];

int main(void) {
    init();   // For Arduino.

    Serial.begin(9600);
    Wire.begin();
    BMA180 myAccel(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyro(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, and 2000 deg/s

    for (;;) {
        myAccel.Poll();
        myGyro.Poll();

        delay(250);
    }

    return 0;
}

