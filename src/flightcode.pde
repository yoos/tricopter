#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "imu.cpp"

//#define REPORT_MOTORVAL

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
    unsigned long nextRuntime = millis();
    loopCount = 0;

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    for (int i=0; i<3; i++) {
        motor[i].write(0);
        motorVal[i] = 0;
    }


    for (;;) {
        if (millis() >= nextRuntime) {
            myIMU.Update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
            nextRuntime += SYSINTRV;   // Update next loop start time.

            // Telemetry loop.
            if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                sw(armed);
                sw(0xf0); sw(0xf0);
            }

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
                // sp("System: Motors not armed.\n");
                if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                    sw("_");
                    sw(0xf0); sw(0xf0);
                }
                for (int i=0; i<3; i++) {
                    motor[i].write(TMIN);   // Disregard what Pilot says and write TMIN.
                    #ifdef REPORT_MOTORVAL
                    if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                        sw(motorVal[i]);   // ..and hopefully Pilot has sense and won't be trying to do anything dangerous.
                    }
                    #endif
                }
                tailServo.write(90);

                #ifdef REPORT_MOTORVAL
                if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                    sw(tailServoVal);
                    sw(0xf0); sw(0xf0);
                }
                #endif

                // Check that motor values set by Pilot are within the arming threshold.
                if (abs(motorVal[MT] - TMIN) < MOTOR_ARM_THRESHOLD &&
                    abs(motorVal[MR] - TMIN) < MOTOR_ARM_THRESHOLD &&
                    abs(motorVal[ML] - TMIN) < MOTOR_ARM_THRESHOLD) {
                    armed++;   // Once Pilot shows some sense, arm.
                    if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                        sw("A");
                        sw(0xf0); sw(0xf0);
                    }
                }
            }
            else if (triWatchdog.isAlive) {   // ..then check if Watchdog is alive.
                // motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs .ints

                if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                    sw("!");
                    sw(0xf0); sw(0xf0);
                }
                for (int i=0; i<3; i++) {
                    motor[i].write(motorVal[i]);   // Write motor values to motors.
                    #ifdef REPORT_MOTORVAL
                    if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                        sw(motorVal[i]);
                    }
                    #endif
                }
                tailServo.write(tailServoVal);
                #ifdef REPORT_MOTORVAL
                if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                    sw(tailServoVal);
                    sw(0xf0); sw(0xf0);
                }
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

            // Datafeed to serialmon.py for visualization.
            if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                sp("DCM");   // Index tag 'DCM'.
                for (int i=0; i<3; i++) {
                    for (int j=0; j<3; j++) {
                        //sp(gyroDCM[i][j]);
                        //sp(" ");
                        sw((byte*) &gyroDCM[i][j], 4);
                    }
                }
                sw(0xf0); sw(0xf0);
            }

            if (loopCount % TELEMETRY_REST_INTERVAL == 0) {
                sp(nextRuntime);
                sw(0xf0); sw(0xf0);
                sp(millis());
                sw(0xf0); sw(0xf0);
                sp((int) (millis() - (nextRuntime - SYSINTRV)));
                sw(0xde); sw(0xad); sw(0xbe); sw(0xef);
            }

            loopCount++;
            loopCount = loopCount % 1000;
        }
    }

    return 0;
}

