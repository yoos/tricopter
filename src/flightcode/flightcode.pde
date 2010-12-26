#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "watchdog.cpp"
#include "system.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Introduce crew.
    Pilot Yeager;
    Watchdog Jasper(DOGLIFE);   // Timeout in ms.

    // Start system.
    System Tric;

    for (;;) {
        Yeager.Listen();
        Jasper.Watch(Yeager.hasFood);
//      myGyr.Poll();
//      myAcc.Poll();
        if (Jasper.isAlive) {
            Yeager.Fly(Tric);
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


