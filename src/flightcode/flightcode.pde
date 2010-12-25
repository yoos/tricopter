#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "pilot.cpp"
#include "system.cpp"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Begin system services.
    Communicator Alice;
    Watchdog Jasper(DOGLIFE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.
    Pilot Yeager;
    System Tric;
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
            Yeager.Fly(Alice.input, Tric.motorVal);
            Tric.Run();
//          myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
            delay(100);   // TODO: Once everything's done, make system run at 40 Hz.
        }
        while (!Jasper.isAlive) {
            Tric.Die();
        }
    }

    return 0;
}


