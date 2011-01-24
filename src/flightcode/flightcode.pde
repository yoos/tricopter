#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "system.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
//  pinMode(22, OUTPUT);
//  digitalWrite(22, LOW);
//  pinMode(23, OUTPUT);
//  digitalWrite(23, LOW);
//  pinMode(24, OUTPUT);
//  digitalWrite(24, LOW);
    
    // Introduce crew.
    Pilot Yeager;
    Watchdog Jasper(DOGLIFE);   // Timeout in ms.

    // Start system.
    System Tric;

    for (;;) {
        Yeager.Listen();
        Jasper.Watch(Yeager.hasFood);
        if (Jasper.isAlive) {
            Yeager.Fly(Tric);
            Tric.Run();
//          myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
        }
        else {
            Tric.Die();
        }
        delay(25);   // TODO: Once everything's done, make system run at 40 Hz.
    }

    return 0;
}


