/*! \file flight_controller.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Main loop.
 *
 *  Details.
 */

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
    Pilot pilot;
    Watchdog watchdog(DOGLIFE);   // Timeout in ms.

    // Start system.
    IMU imu;
    imu.init();

    Servo pwmDevice[4];
    pwmDevice[SERVO_T].attach(PST);

    Timer3.initialize(2500);   // 400 Hz PWM.

    // Variables

    armCount = TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // We will subtract 1 from the armed value every time we receive the arming numbers from comm. Arming numbers will have to be sent for a total of TIME_TO_ARM (in ms) before this variable reaches 0.
    unsigned long nextRuntime = micros();
    loopCount = 0;

    for (;;) {
        if (micros() >= nextRuntime) {
            // ================================================================
            // System loop
            // ================================================================
            imu.update();   // Run this ASAP when loop starts so gyro integration is as accurate as possible.
            nextRuntime += MASTER_DT;   // Update next loop start time.

            // ================================================================
            // Control loop
            // ================================================================
            if (loopCount % CONTROL_LOOP_INTERVAL == 0) {
                watchdog.watch(pilot.hasFood);

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
                    Timer3.pwm(PMT, TMIN);
                    Timer3.pwm(PMR, TMIN);
                    Timer3.pwm(PML, TMIN);
                    pwmDevice[SERVO_T].writeMicroseconds(SERVO_US_ZERO);

                    // Check that motor values set by Pilot are within the
                    // arming threshold.
                    if (abs(pwmOut[MOTOR_T] - (TMIN + MOTOR_T_OFFSET)) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_R] - (TMIN + MOTOR_R_OFFSET)) < MOTOR_ARM_THRESHOLD &&
                        abs(pwmOut[MOTOR_L] - (TMIN + MOTOR_L_OFFSET)) < MOTOR_ARM_THRESHOLD) {
                        armCount--;   // Once Pilot shows some sense, arm.
                    }
                }
                else if (watchdog.isAlive) {   // ..then check if Watchdog is alive.
                    Timer3.pwm(PMT, pwmOut[MOTOR_T]);
                    Timer3.pwm(PMR, pwmOut[MOTOR_R]);
                    Timer3.pwm(PML, pwmOut[MOTOR_L]);
                    pwmDevice[SERVO_T].writeMicroseconds(pwmOut[SERVO_T]);
                }
                else {   // ..otherwise, die.
                    // pilot.Abort();   // TODO: Do I even need this anymore?
                    if (armCount <= 0)
                        armCount = (int) TIME_TO_ARM/(MASTER_DT*CONTROL_LOOP_INTERVAL);   // Set armed status back off.
                    for (int i=0; i<3; i++) {
                        pwmOut[i] = TMIN;
                    }
                    Timer3.pwm(PMT, TMIN);
                    Timer3.pwm(PMR, TMIN);
                    Timer3.pwm(PML, TMIN);
                    pwmDevice[SERVO_T].writeMicroseconds(SERVO_US_ZERO);
                }
            }

            // ================================================================
            // Communications loop
            // ================================================================
            if (loopCount % COMM_LOOP_INTERVAL == 0) {
                pilot.fly();

                #ifdef SEND_ARM_STATUS
                sendArmStatus();
                #endif
            }
            else {
                /* The pilot communicates with base and updates pwmOut
                 * according to the joystick axis values it receives. */
                pilot.listen();
            }

            if (loopCount % COMM_LOOP_INTERVAL == 1) {
                #ifdef SEND_DCM
                sendDCM();
                #endif
            }
            else if (loopCount % COMM_LOOP_INTERVAL == 2) {
                #ifdef SEND_TARGET_ROTATION
                sendTargetRotation();
                #endif

                #ifdef SEND_MOTOR_VALUES
                sendMotorValues();
                #endif
            }
            else if (loopCount % COMM_LOOP_INTERVAL == 3) {
                #ifdef SEND_PID_DATA
                sendPIDData();
                #endif
            }
            else if (loopCount % COMM_LOOP_INTERVAL == 4) {
                sendTelemetryEnd(nextRuntime);
            }

            // Transmit data.
            sendTelemetry();

            loopCount++;
            loopCount = loopCount % 100;
        } // endif
    } // endfor

    return 0;
}

