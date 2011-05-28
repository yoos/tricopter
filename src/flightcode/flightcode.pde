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

    unsigned long nextRuntime = 0;

    for (;;) {
        if (millis() >= nextRuntime) {
            nextRuntime += SYSINTRV;   // Increment by DT.
            if (Jasper.isAlive) {
                Tric.Run();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
                Yeager.Fly(Tric);
//              myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
            }
            else {
                Tric.Die();
            }
            Yeager.Listen();
            Jasper.Watch(Yeager.hasFood);
            Serial.println(millis());
        }
    }

    return 0;
}


