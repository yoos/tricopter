#ifndef GLOBALS_H
#define GLOBALS_H

// #define DEBUG

/*****************************************************************************
 * Variables
 *****************************************************************************/

int armed;
float motorVal[3], tailServoVal;
char commStr[250];   // String to be sent out to base.
float commandPitch;   // Pitch command to be processed through PID.
float commandRoll;   // Roll command to be processed through PID.
float currentDCM[3][3];   // Current position DCM calculated by IMU.
float targetDCM[3][3];   // Target position DCM calculated by Pilot.
float currentAngle[2];   // TEST: Current angle.
float targetAngle[2];   // TEST: Target angle.
float gVal[3];   // [-2000,2000] deg/s mapped to [-1,1]


/*****************************************************************************
 * Serial: everything that has to do with TX/RX.
 *****************************************************************************/

#define BAUDRATE 115200   // Fiddle with this. The XBee sometimes seems to have trouble with high baudrates like 57600.

#define SERHEAD    255   // Serial header byte. Pilot interprets the four bytes following this header byte as motor commands.
#define PACKETSIZE 4     // Each packet contains header plus X, Y, Twist, and Z.
#define INPUT_MIN  1     // Minimum integer input value from joystick.
#define INPUT_MAX  251   // Maximum integer input value from joystick.
#define SX 0   // Serial byte location for joystick X axis.
#define SY 1   // Serial byte location for joystick Y axis.
#define ST 2   // Serial byte location for joystick T (twist) axis.
#define SZ 3   // Serial byte location for joystick Z axis.


/*****************************************************************************
 * Software configuration: any parameter that is purely code-related or is
 * relatively frequently changed.
 *****************************************************************************/

#define SYSINTRV 10   // System loop interval in milliseconds.
#define DOGLIFE 300   // Watchdog life in milliseconds.

#define DCM_COEFF 90   // Scale current-to-target DCM difference.
#define GYRO_COEFF 15   // Try to stabilize craft.
//#define ACCEL_COEFF 90   // TEST: Try to stabilize craft.
#define TMIN 16   // Servo signal that registers as minimum throttle to ESC.
#define TMAX 90   // Servo signal that registers as maximum throttle to ESC.
#define TIME_TO_ARM 2000   // This divided by SYSINTRV determines how long it takes to arm the system.
#define MOTOR_ARM_THRESHOLD 3   // This is added to TMIN to determine whether or not to arm the system.

#define MT 0   // Tail motor array index.
#define MR 1   // Right motor array index.
#define ML 2   // Left motor array index.


/*****************************************************************************
 * Hardware configuration: any parameter that is changed so infrequently that
 * it may as well be hard-coded.
 *****************************************************************************/

#define MOTOR_T_OFFSET 3   // Speed offset for tail motor.
#define MOTOR_R_OFFSET 0   // Speed offset for right motor.
#define MOTOR_L_OFFSET 0   // Speed offset for left motor.
#define MOTOR_T_SCALE  1.03   // Scale speed of tail motor.
#define MOTOR_R_SCALE  1   // Scale speed of right motor.
#define MOTOR_L_SCALE  1   // Scale speed of left motor.
#define TAIL_SERVO_DEFAULT_POSITION 50

#define PMT 4   // Tail motor pin.
#define PMR 2   // Right motor pin.
#define PML 3   // Left motor pin.
#define PST 5   // Tail servo pin.


/*****************************************************************************
 * Flight modes: not yet implemented.
 *****************************************************************************/

//#define OFF 0
//#define IDLE 1
//#define HOVER 2
//#define ACRO 3
//#define AUTO 4
//#define AUTO_HOVER 5


/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define PITCH 0   // X axis index for PID struct.
#define ROLL 1   // Y axis index for PID struct.


/*****************************************************************************
 * Constants
 *****************************************************************************/

#define PI 3.141592653589793238462643383279502884197f


/*****************************************************************************
 * Functions
 *****************************************************************************/

void zeroStr(char *sStr) {
    for (int i=0; i<sizeof(sStr); i++) {
        sStr[i] = 0;
    }
}

// Basic print function from http://www.arduino.cc/playground/Main/Printf
void p(char *fmt, ... ){
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(tmp, 128, fmt, args);
    va_end (args);
    Serial.print(tmp);
}

#endif // GLOBALS_H

