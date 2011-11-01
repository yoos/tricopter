#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "imu.cpp"

// #define REPORT_MOTORVAL

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

    armed = (int) -(TIME_TO_ARM/SYSINTRV-1);   // Pilot will add 1 to the armed value every time it receives the arming numbers from comm. Arming numbers will have to be sent for a total of 2000 ms before this variable is positive (and therefore true).
    unsigned long nextRuntime = 0;
    teleCount = 0;

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    for (int i=0; i<3; i++) {
        motor[i].write(0);
        motorVal[i] = 0;
    }


    for (;;) {
        if (millis() >= nextRuntime) {
            if (teleCount == TELEMETRY_REST_INTERVAL) teleCount = 0;
            else teleCount++;

            serPrint("%d ", armed);
            //serPrint("(%d) ", nextRuntime);
            nextRuntime += SYSINTRV;   // Increment by DT.
            myIMU.Update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.

            //serPrint("(%f, %f, %f) ", targetDCM[2][0], targetDCM[2][1], targetDCM[2][2]);
            //serPrint("(%f, %f, %f) ", currentDCM[2][0], currentDCM[2][1], currentDCM[2][2]);

            triPilot.Listen();
            /* The pilot communicates with base and updates motorVal and 
             * tailServoVal according to the joystick axis values it 
             * receives. */
            triPilot.Fly();
            triWatchdog.Watch(triPilot.hasFood);

            /* Don't run system unless armed!
             * Pilot will monitor serial inputs and update System::motorVal[]. System 
             * will send ESCs a "nonsense" value of 0 until it sees that all three 
             * motor values are zerod. It will then consider itself armed and start 
             * sending proper motor values.
             */
            if (armed < 1) {   // First check that system is armed.
                // serPrint("System: Motors not armed.\n");
                serPrint("_ ");
                for (int i=0; i<3; i++) {
                    motor[i].write(TMIN);   // Disregard what Pilot says and write TMIN.
                    #ifdef REPORT_MOTORVAL
                    serPrint("%d ", motorVal[i]);   // ..and hopefully Pilot has sense and won't be trying to do anything dangerous.
                    #endif
                }
                tailServo.write(90);

                #ifdef REPORT_MOTORVAL
                serPrint("%d ", tailServoVal);
                #endif

                // Check that motor values set by Pilot are within the arming threshold.
                if (abs(motorVal[MT] - TMIN) < MOTOR_ARM_THRESHOLD && 
                    abs(motorVal[MR] - TMIN) < MOTOR_ARM_THRESHOLD && 
                    abs(motorVal[ML] - TMIN) < MOTOR_ARM_THRESHOLD) {
                    armed++;   // Once Pilot shows some sense, arm.
                    serPrint("A ");
                }
            }
            else if (triWatchdog.isAlive) {   // ..then check if Watchdog is alive.
                // motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs .ints

                serPrint("! ");
                for (int i=0; i<3; i++) {
                    motor[i].write(motorVal[i]);   // Write motor values to motors.
                    #ifdef REPORT_MOTORVAL
                    serPrint("%d ", motorVal[i]);
                    #endif
                }
                tailServo.write(tailServoVal);
                #ifdef REPORT_MOTORVAL
                serPrint(tailServoVal);
                #endif
            }
            else {   // ..otherwise, die.
                // triPilot.Abort();   // TODO: Do I even need this anymore?
                if (armed >= 1)
                    armed = (int) -(TIME_TO_ARM/SYSINTRV-1);   // Set armed status back off.
                for (int i=0; i<3; i++) {
                    motorVal[i] -= 5;   // Slowly turn off.
                    if (motorVal[i] < 0) motorVal[i] = 0;
                    motor[i].write(motorVal[i]);
                }
                tailServoVal = 90;
                tailServo.write(tailServoVal);
            }

            serPrint(" (%d, %d, %f)  ", commandPitch, commandRoll, gVal[0]);

            //serPrint("(%f, %f) ", targetAngle[0], targetAngle[1]);
            //serPrint("(%f, %f) ", currentAngle[0], currentAngle[1]);
            serPrint("\n");   // Send newline after every system iteration. TODO: Eventually implement a Telemetry class.
        }
    }

    return 0;
}




