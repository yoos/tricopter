#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "system.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Introduce crew.
    Communicator Alice;
    Watchdog Jasper(DOGLIFE);   // Timeout in ms.
    Pilot Yeager;

    // Start system.
    System Tric;

    for (;;) {
        Alice.Listen();
        Jasper.Watch(Alice.hasFood);
//      myGyr.Poll();
//      myAcc.Poll();
        if (Jasper.isAlive) {
            Yeager.Fly(Alice.input, Tric.motorVal);
            Tric.Run();
//          myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
        }
        else {
            Tric.Die();
        }
        delay(100);   // TODO: Once everything's done, make system run at 40 Hz.
    }

    return 0;
}


