#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

char genStr[512];

int main(void) {
    init();   // For Arduino.
 
    Serial.begin(9600);
    Serial.println("Antares starting!");
    Wire.begin();
    Watchdog Jasper(3000, DOGBONE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.

    for (;;) {
        while (Jasper.isAlive) {
            Jasper.watch();
            myGyr.Poll();
            myAcc.Poll();
            delay(250);   // Eventually change this to 25 ms for 40 Hz operation.
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


