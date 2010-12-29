#ifndef GLOBALS_H
#define GLOBALS_H


// Configurables

#define DEBUG
#define PACKETSIZE 4   // Each packet contains header plus X, Y, Twist, and Z.
#define INPUT_MIN 0   // Minimum integer input value from joystick
#define INPUT_MAX 250   // Maximum integer input value from joystick
#define THROTTLE_MIN 16   // Servo signal that registers as minimum signal to ESC.
#define THROTTLE_MAX 170   // Servo signal that registers as maximum signal to ESC.

#define BAUDRATE 9600
#define IMU_SAMPLE_INTERVAL 25   // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1

#define SERHEAD 255
#define DOGBONE 254
#define DOGLIFE 500   // Watchdog life in milliseconds


// BMA180 config
#define RANGEMASK 0x0E
#define BWMASK 0xF0


// Flight modes

#define OFF 0
#define IDLE 1
#define HOVER 2
#define ACRO 3
#define AUTO 4
#define AUTO_HOVER 5

// Digital pins

#define MOTOR_T 10 // Tail motor PWM
#define MOTOR_L 11 // Left motor PWM
#define MOTOR_R 12 // Right motor PWM

#define SERVO_T 13 // Tail servo PWM for yaw control

// Constants

#define PI 3.1415926535

void zeroStr(char *sStr) {
    for (int i=0; i<128; i++) {
        sStr[i] = 0;
    }
}










#endif // GLOBALS_H

