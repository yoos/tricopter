#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "imu.cpp"

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

    Servo pwmDevice[4];
    pwmDevice[MOTOR_T].attach(PMT);
    pwmDevice[MOTOR_R].attach(PMR);
    pwmDevice[MOTOR_L].attach(PML);
    pwmDevice[SERVO_T].attach(PST);
    
    // Variables

    armCount = (int) TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // We will subtract 1 from the armed value every time we receive the arming numbers from comm. Arming numbers will have to be sent for a total of TIME_TO_ARM (in ms) before this variable reaches 0.
    unsigned long nextRuntime = millis();
    loopCount = 0;

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    for (int i=0; i<3; i++) {
        pwmDevice[i].write(0);
        pwmOut[i] = 0;
    }


    for (;;) {
        if (millis() >= nextRuntime) {
            // ================================================================
            // System loop
            // ================================================================

            myIMU.Update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
            nextRuntime += MASTER_DT;   // Update next loop start time.


            // ================================================================
            // Control loop
            // ================================================================

            if (loopCount % CONTROL_LOOP_INTERVAL == 0) {
                triPilot.Listen();
                /* The pilot communicates with base and updates pwmOut
                 * according to the joystick axis values it receives. */
                triPilot.Fly();
                triWatchdog.Watch(triPilot.hasFood);

                /* Don't run system unless armed!
                 * Pilot will monitor serial inputs and update
                 * System::pwmOut[]. System will send ESCs a "nonsense" value
                 * of 0 until it sees that all three motor values are zerod. It
                 * will then consider itself armed and start sending proper
                 * motor values.
                 */
                if (armCount > 0) {   // First check that system is armed.
                    // sp("System: Motors not armed.\n");
                    pwmDevice[MOTOR_T].write(TMIN);   // Disregard what Pilot says and write TMIN.
                    pwmDevice[MOTOR_R].write(TMIN);   // Disregard what Pilot says and write TMIN.
                    pwmDevice[MOTOR_L].write(TMIN);   // Disregard what Pilot says and write TMIN.
                    pwmDevice[SERVO_T].write(90);

                    // Check that motor values set by Pilot are within the
                    // arming threshold.
                    if (abs(pwmOut[MOTOR_T] - TMIN) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_R] - TMIN) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_L] - TMIN) < MOTOR_ARM_THRESHOLD) {
                        armCount--;   // Once Pilot shows some sense, arm.
                    }
                }
                else if (triWatchdog.isAlive) {   // ..then check if Watchdog is alive.
                    for (int i=0; i<4; i++) {
                        pwmDevice[i].write(pwmOut[i]);   // Write motor values to motors.
                    }
                }
                else {   // ..otherwise, die.
                    // triPilot.Abort();   // TODO: Do I even need this anymore?
                    if (armCount <= 0)
                        armCount = (int) TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // Set armed status back off.
                    for (int i=0; i<3; i++) {
                        pwmOut[i] -= 3;   // Slowly turn off.
                        if (pwmOut[i] < 0) pwmOut[i] = 0;
                        pwmDevice[i].write(pwmOut[i]);
                    }
                    pwmOut[SERVO_T] = 90;
                    pwmDevice[SERVO_T].write(pwmOut[SERVO_T]);
                }
            }


            // ================================================================
            // Telemetry loop
            // ================================================================

            if (loopCount % TELEMETRY_LOOP_INTERVAL == 0) {
                // ============================================================
                // Report arm status.
                // ============================================================
                sw(armCount);
                //if (armed < 1) {   // First check that system is armed.
                //    sw("_");
                //}
                //else if (triWatchdog.isAlive) {   // ..then check if Watchdog is alive.
                //    sw("!");
                //}
                sw(0xff); sw(0xff);

                // ============================================================
                // Report target rotation vector (in BODY frame). TODO:
                // targetRot is a misleading name because this is dwB (delta
                // omega BODY), so to speak (i.e., an update to the DCM). There
                // is a target DCM we could calculate but is unneeded at this
                // moment.
                // ============================================================
                //sw("ROT");
                //for (int i=0; i<3; i++) {
                //    sw((byte*) &targetRot[i], 4);
                //}
                //sw(0xff); sw(0xff);

                // ============================================================
                // Report motor values.
                // ============================================================
                sw("MTR");
                for (int i=0; i<4; i++) {
                    sw((byte) pwmOut[i]);
                    //sw((byte*) &pwmOut[i], 4);
                }
                sw(0xff); sw(0xff);

                // ============================================================
                // Datafeed to serialmon.py for visualization.
                // ============================================================
                sw("DCM");   // Index tag 'DCM'.
                for (int i=0; i<3; i++) {
                    for (int j=0; j<3; j++) {
                        //sw((byte*) &gyroDCM[i][j], 4);
                        sw((byte) (250*(gyroDCM[i][j]+1)/2+1));
                    }
                }
                sw(0xff); sw(0xff);

                // ============================================================
                // Report loop time.
                // ============================================================
                //sp(nextRuntime);
                //sw(0xff); sw(0xff);
                //sp(millis());
                //sw(0xff); sw(0xff);
                sp((int) (millis() - (nextRuntime - MASTER_DT)));
                sw(0xde); sw(0xad); sw(0xbe); sw(0xef);
            }

            loopCount++;
            loopCount = loopCount % 1000;
        } // endif
    } // endfor

    return 0;
}

