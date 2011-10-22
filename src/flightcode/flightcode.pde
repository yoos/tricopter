#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "imu.cpp"

#define REPORT_MOTORVAL

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Introduce crew.
    Pilot triPilot;
    Watchdog triWatchdog(DOGLIFE);   // Timeout in ms.

    // Start system.
    IMU myIMU;
    myIMU.Init();

    Servo motor[3], tailServo;
    motor[MT].attach(PMT);
    motor[MR].attach(PMR);
    motor[ML].attach(PML);
    tailServo.attach(PST);
    
    // Variables

    armed = -2000/SYSINTRV;   // Pilot will add 1 to the armed value every time it receives the arming numbers from comm. Arming numbers will have to be sent for a total of 2000 ms before this variable is positive (and therefore true).
    unsigned long nextRuntime = 0;

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    for (int i=0; i<3; i++) {
        motor[i].write(0);
        motorVal[i] = 0;
    }


    for (;;) {
        if (millis() >= nextRuntime) {
            nextRuntime += SYSINTRV;   // Increment by DT.
            myIMU.Update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
            // Serial.println(millis());

            /* Don't run system unless armed!
             * Pilot will monitor serial inputs and update System::motorVal[]. System 
             * will send ESCs a "nonsense" value of 0 until it sees that all three 
             * motor values are zerod. It will then consider itself armed and start 
             * sending proper motor values.
             */
            if (!armed) {
                // Serial.println("System: Motors not armed.");
                #ifdef REPORT_MOTORVAL
                Serial.print("_ ");
                #endif

                for (int i=0; i<3; i++) {
                    motor[i].write(TMIN);
                    #ifdef REPORT_MOTORVAL
                    Serial.print(motorVal[i]);
                    Serial.print(" ");
                    #endif
                }
                tailServo.write(90);

                #ifdef REPORT_MOTORVAL
                Serial.print(tailServoVal);
                #endif

                if (motorVal[MT] == TMIN && 
                    motorVal[MR] == TMIN && 
                    motorVal[ML] == TMIN) {
                    armed++;
                    Serial.println("System: Motors armed.");
                }
            }
            else if (triWatchdog.isAlive) {
                #ifdef REPORT_MOTORVAL
                Serial.print("! ");
                #endif

                /* The pilot communicates with base and updates motorVal and 
                 * tailServoVal according to the joystick axis values it 
                 * receives. */
                triPilot.Fly();

                // motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs .ints

                for (int i=0; i<3; i++) {
                    motor[i].write(motorVal[i]);   // Write motor values to motors.
                    #ifdef REPORT_MOTORVAL
                    Serial.print(motorVal[i]);
                    Serial.print(" ");
                    #endif
                }
                tailServo.write(tailServoVal);
                #ifdef REPORT_MOTORVAL
                Serial.print(tailServoVal);
                #endif
            }
            else {
                // triPilot.Abort();   // TODO: Do I even need this anymore?
                for (int i=0; i<3; i++) {
                    motorVal[i] -= 10;   // Slowly turn off.
                    if (motorVal[i] < 0) motorVal[i] = 0;
                    motor[i].write(motorVal[i]);
                }
                
            }

            triPilot.Listen();
            triWatchdog.Watch(triPilot.hasFood);
            Serial.println("");   // Send newline after every system iteration. TODO: Eventually implement a Telemetry class.
        }
    }

    return 0;
}


