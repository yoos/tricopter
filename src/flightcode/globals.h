#ifndef GLOBALS_H
#define GLOBALS_H


// Configurables

// #define DEBUG
#define PACKETSIZE 4   // Each packet contains header plus X, Y, Twist, and Z.
#define INPUT_MIN 1   // Minimum integer input value from joystick
#define INPUT_MAX 251   // Maximum integer input value from joystick
#define TMIN 16   // Servo signal that registers as minimum throttle to ESC.
#define TMAX 180   // Servo signal that registers as maximum throttle to ESC.

#define BAUDRATE 57600
#define IMU_SAMPLE_INTERVAL 25   // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1

#define SYSINTRV 25   // System run interval in milliseconds
#define SERHEAD 255
#define DOGBONE 254
#define DOGLIFE 500   // Watchdog life in milliseconds



// Serial axis array index values
#define SX 0
#define SY 1
#define ST 2
#define SZ 3

// Motors array index values
#define MT 0   // Tail motor
#define MR 1   // Right motor
#define ML 2   // Left motor

// IMU constants
#define KH 0
#define DT 0

// Pins
#define PMT 2   // Tail motor
#define PMR 3   // Right motor
#define PML 4   // Left motor
#define PST 5   // Tail servo

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


// Constants

#define PI 3.1415926535

void zeroStr(char *sStr) {
    for (int i=0; i<128; i++) {
        sStr[i] = 0;
    }
}


#endif // GLOBALS_H

