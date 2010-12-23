#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

char motorInput[32];

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Serial.begin(9600);
    Wire.begin();
    
    // Begin system services.
    Serial.println("Antares starting!");
    Watchdog Jasper(3000, DOGBONE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.
    Servo myServo;

    // Attach motors.
    myServo.attach(9);

    for (;;) {
        while (Jasper.isAlive) {
            Jasper.watch();
            myGyr.Poll();
            myAcc.Poll();

            int i = 0;
            while (Serial.available() > 0) {
                motorInput[i] = Serial.read();
                i++;
            }
            if (i > 0) {
                Serial.println(motorInput);
                myServo.write(atoi(motorInput));
                for (int i=0; i<32; i++) {
                    motorInput[i] = 0;
                }
            }
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


