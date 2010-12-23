#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Begin system services.
    Communicator Alice;
    Alice.Send("Antares starting!");
    Watchdog Jasper(3000, DOGBONE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.
    Servo myServo;

    // Attach motors.
    myServo.attach(MOTOR_L);

    char motorInput[32];
    for (;;) {
//      while (Jasper.isAlive) {
        while (true) {
            Jasper.Watch();
//          myGyr.Poll();
//          myAcc.Poll();


            char myChr;
            if (Serial.available()) {
                myChr = Serial.read();
            }

            if (myChr == DOGBONE) {
                Jasper.Feed();
            }

            if (myChr == SERHEAD) {
                while (Serial.available() > 0) {
                    for (int i=0; i<2; i++) {
                        motorInput[i] = Serial.read();
                    }
                }
                Serial.println(motorInput);
                myServo.write(motorInput[0]);
            }
            delay(100);
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


