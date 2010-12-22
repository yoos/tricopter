#include "itg3200.cpp"
#include "bma180.cpp"

char genStr[512];

int main(void) {
    init();   // For Arduino.

    Serial.begin(9600);
    Wire.begin();
    BMA180 myAccel(0x04, 0x02);   // range, bandwidth: DS p. 27
    ITG3200 myGyro;

    for (;;) {
//      myAccel.Poll();
        myGyro.Poll();

        delay(250);
    }

    return 0;
}

