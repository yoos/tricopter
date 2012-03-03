/*! \file flight_controller.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Main loop.
 *
 *  Details.
 */

#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "pilot.cpp"
#include "watchdog.cpp"
#include "imu.cpp"
#include "telemetry.h"

int main(void) {
    init();   // For Arduino.

    // Begin Arduino services.
    Wire.begin();

    // Introduce crew.
    Pilot triPilot;
    Watchdog triWatchdog(DOGLIFE);   // Timeout in ms.

    // Start system.
    IMU myIMU;
    myIMU.init();

    Servo pwmDevice[4];
    pwmDevice[MOTOR_T].attach(PMT);
    pwmDevice[MOTOR_R].attach(PMR);
    pwmDevice[MOTOR_L].attach(PML);
    pwmDevice[SERVO_T].attach(PST);

    // Variables

    armCount = TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // We will subtract 1 from the armed value every time we receive the arming numbers from comm. Arming numbers will have to be sent for a total of TIME_TO_ARM (in ms) before this variable reaches 0.
    unsigned long nextRuntime = micros();
    loopCount = 0;

    // Write 0 to motors to prevent them from spinning up upon Seeeduino reset!
    for (int i=0; i<3; i++) {
        pwmDevice[i].writeMicroseconds(0);
        pwmOut[i] = 0;
    }


    for (;;) {
        if (micros() >= nextRuntime) {
            // ================================================================
            // System loop
            // ================================================================
            myIMU.update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
            nextRuntime += MASTER_DT;   // Update next loop start time.

            // ================================================================
            // Control loop
            // ================================================================
            if (loopCount % CONTROL_LOOP_INTERVAL == 0) {
                /* The pilot communicates with base and updates pwmOut
                 * according to the joystick axis values it receives. */
                triPilot.fly();
                triWatchdog.watch(triPilot.hasFood);

                /* Don't run system unless armed!
                 * Pilot will monitor serial inputs and update
                 * System::pwmOut[]. System will send ESCs a "nonsense" value
                 * of 0 until it sees that all three motor values are zerod. It
                 * will then consider itself armed and start sending proper
                 * motor values.
                 */
                if (armCount > 0) {   // First check that system is armed.
                    // sp("System: Motors not armed.\n");

                    // Disregard what Pilot says and write TMIN.
                    pwmDevice[MOTOR_T].writeMicroseconds(TMIN);
                    pwmDevice[MOTOR_R].writeMicroseconds(TMIN);
                    pwmDevice[MOTOR_L].writeMicroseconds(TMIN);
                    pwmDevice[SERVO_T].writeMicroseconds(SERVO_US_ZERO);

                    // Check that motor values set by Pilot are within the
                    // arming threshold.
                    if (abs(pwmOut[MOTOR_T] - (TMIN + MOTOR_T_OFFSET)) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_R] - (TMIN + MOTOR_R_OFFSET)) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_L] - (TMIN + MOTOR_L_OFFSET)) < MOTOR_ARM_THRESHOLD) {
                        armCount--;   // Once Pilot shows some sense, arm.
                    }
                }
                else if (triWatchdog.isAlive) {   // ..then check if Watchdog is alive.
                    for (int i=0; i<4; i++) {
                        pwmDevice[i].writeMicroseconds(pwmOut[i]);   // Write motor values to motors.
                    }
                }
                else {   // ..otherwise, die.
                    // triPilot.Abort();   // TODO: Do I even need this anymore?
                    if (armCount <= 0)
                        armCount = (int) TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // Set armed status back off.
                    for (int i=0; i<3; i++) {
                        pwmOut[i] -= 5;   // Slowly turn off.
                        if (pwmOut[i] < TMIN) pwmOut[i] = TMIN;
                        pwmDevice[i].writeMicroseconds(pwmOut[i]);
                    }
                    pwmDevice[SERVO_T].writeMicroseconds(SERVO_US_ZERO);
                }
            }

            // ================================================================
            // Communications loop
            // ================================================================
            if (loopCount % COMM_LOOP_INTERVAL == 0) {
                triPilot.listen();
            }

            if (loopCount % COMM_LOOP_INTERVAL == 1) {
                #ifdef SEND_ARM_STATUS
                sendArmStatus();
                #endif

                #ifdef SEND_TARGET_ROTATION
                sendTargetRotation();
                #endif

                #ifdef SEND_MOTOR_VALUES
                sendMotorValues();
                #endif
            }

            if (loopCount % COMM_LOOP_INTERVAL == 2) {
                #ifdef SEND_DCM
                sendDCM();
                #endif
            }

            if (loopCount % COMM_LOOP_INTERVAL == 3) {
                #ifdef SEND_PID_DATA
                sendPIDData();
                #endif
            }

            if (loopCount % COMM_LOOP_INTERVAL == 4) {
                sendTelemetryEnd(nextRuntime);
            }

            loopCount++;
            loopCount = loopCount % 1000;
        } // endif
    } // endfor

    return 0;
}

