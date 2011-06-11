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
    
    // Introduce crew.
    Pilot triPilot;
    Watchdog triWatchdog(DOGLIFE);   // Timeout in ms.

    // Start system.
    System triSystem;

    unsigned long nextRuntime = 0;

    for (;;) {
        if (millis() >= nextRuntime) {
            nextRuntime += SYSINTRV;   // Increment by DT.
            if (triWatchdog.isAlive) {
                triSystem.Run();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
                // Serial.println(millis());
                triPilot.Fly(triSystem);
//              myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
            }
            else {
                triSystem.Die();
            }
            triPilot.Listen();
            triWatchdog.Watch(triPilot.hasFood);
        }
    }

    return 0;
}


