#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Begin system services.
    Communicator Alice;
    Pilot Yeager;
    Watchdog Jasper(DOGLIFE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.
    Servo myServo;

    // Attach motors.
    myServo.attach(9);

    for (;;) {
//      while (Jasper.isAlive) {
        Jasper.isAlive = true;
        int n = 0;
        while (true) {
            Alice.Listen();
            Jasper.Watch(Alice.hasFood);
//          myGyr.Poll();
//          myAcc.Poll();
            Yeager.Fly(Alice.input);
//          myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
            delay(100);
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


