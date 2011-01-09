#ifndef GLOBALS_H
#define GLOBALS_H


// Configurables

// #define DEBUG
#define PACKETSIZE 2   // Each packet contains header plus X, Y, Twist, and Z.
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



// Serial
#define SX 0
#define SY 1
#define ST 2
#define SZ 3

// Motors
#define MT 0
#define MR 1
#define ML 2

// IMU
#define KH 0
#define DT 0


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

