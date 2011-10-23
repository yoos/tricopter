#ifndef GLOBALS_H
#define GLOBALS_H

// Variables
int armed;
int motorVal[3], tailServoVal;
float currentDCM[3][3];   // Current position DCM calculated by IMU.
float targetDCM[3][3];   // Target position DCM calculated by Pilot.


// Configurables

// #define DEBUG
#define PACKETSIZE 4   // Each packet contains header plus X, Y, Twist, and Z.
#define INPUT_MIN 1   // Minimum integer input value from joystick
#define INPUT_MAX 251   // Maximum integer input value from joystick
#define TMIN 16   // Servo signal that registers as minimum throttle to ESC.
#define TMAX 70   // Servo signal that registers as maximum throttle to ESC.
#define TAIL_SERVO_DEFAULT_POSITION 70

#define BAUDRATE 19200
#define IMU_SAMPLE_INTERVAL 10   // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1

#define SYSINTRV 10   // System run interval in milliseconds
#define SERHEAD 255
#define DOGBONE 254
#define DOGLIFE 300   // Watchdog life in milliseconds



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
#define PMT 4   // Tail motor
#define PMR 2   // Right motor
#define PML 3   // Left motor
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

#define PI 3.141592653589793238462643383279502884197f

// Functions

void zeroStr(char *sStr) {
    for (int i=0; i<128; i++) {
        sStr[i] = 0;
    }
}

/* Basic print function from http://www.arduino.cc/playground/Main/Printf */
void p(char *fmt, ... ){
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(tmp, 128, fmt, args);
    va_end (args);
    Serial.print(tmp);
}

#endif // GLOBALS_H

