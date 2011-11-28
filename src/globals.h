#ifndef GLOBALS_H
#define GLOBALS_H

//#include <avr/pgmspace.h>

// #define DEBUG

/*****************************************************************************
 * Variables
 *****************************************************************************/

int armCount;   // Arm status counter.
int loopCount;   // Count system loops.
float pwmOut[4], pwmOutUpdate[4];
char commStr[250];   // String to be sent out to base.
float commandPitch;   // Pitch command to be processed through PID.
float commandRoll;   // Roll command to be processed through PID.
float gyroDCM[3][3];   // Current position DCM calculated by IMU.
float targetRot[3], pidRot[3];
float gVal[3];   // [-2000,2000] deg/s mapped to [-1,1]


// ============================================================================
// PID
//     Match the number of structs to the number of PID value defines.
// ============================================================================
struct PIDdata {
    float P, I, D;
    float lastValue;
    float integratedError;
} PID[4];

//#define PID_MOTOR_T 0
//#define PID_MOTOR_R 1
//#define PID_MOTOR_L 2
//#define PID_SERVO_T 3

#define PID_ROT_X 0
#define PID_ROT_Y 1
#define PID_ROT_Z 2


/*****************************************************************************
 * Serial: everything that has to do with TX/RX.
 *****************************************************************************/

// ============================================================================
// SERIAL IN
// ============================================================================
#define BAUDRATE 57600   // Fiddle with this. The XBee sometimes seems to have trouble with high baudrates like 57600.
#define SERHEAD    255   // Serial header byte. Pilot interprets the four bytes following this header byte as motor commands.
#define PACKETSIZE 4     // Each packet contains header plus X, Y, Twist, and Z.
#define INPUT_MIN  1     // Minimum integer input value from joystick.
#define INPUT_MAX  251   // Maximum integer input value from joystick.
#define SX 0   // Serial byte location for joystick X axis.
#define SY 1   // Serial byte location for joystick Y axis.
#define ST 2   // Serial byte location for joystick T (twist) axis.
#define SZ 3   // Serial byte location for joystick Z axis.

// ============================================================================
// SERIAL OUT
// ============================================================================
#define DCM_SER_TAG 0xfb
#define ROT_SER_TAG 0xfc
#define MOT_SER_TAG 0xfd
#define FIELD_SER_TAG 0xff

/*****************************************************************************
 * Software configuration: any parameter that is purely code-related or is
 * relatively frequently changed.
 *****************************************************************************/

#define MASTER_DT              12   // 12 ms interval = 83 Hz master loop.
#define CONTROL_LOOP_INTERVAL   1   // 1/2 master = 42 Hz. NOTE: This frequency should be HIGHER than comm.py's dataSend frequency!
#define TELEMETRY_LOOP_INTERVAL 5   // 1/5 master = 21 Hz.
#define DOGLIFE 300   // Watchdog life in milliseconds.

//#define DCM_COEFF 90   // Scale current-to-target DCM difference.
//#define GYRO_COEFF 15   // Try to stabilize craft.
//#define ACCEL_COEFF 90   // TEST: Try to stabilize craft.
#define TMIN 16   // Servo signal that registers as minimum throttle to ESC.
#define TMAX 80   // Servo signal that registers as maximum throttle to ESC.
#define TIME_TO_ARM 2000   // This divided by MASTER_DT determines how long it takes to arm the system.
#define MOTOR_ARM_THRESHOLD 3   // This is added to TMIN to determine whether or not to arm the system.

#define MOTOR_T 0   // Tail motor array index.
#define MOTOR_R 1   // Right motor array index.
#define MOTOR_L 2   // Left motor array index.
#define SERVO_T 3   // Tail servo array index.


/*****************************************************************************
 * Hardware configuration: any parameter that is changed so infrequently that
 * it may as well be hard-coded.
 *****************************************************************************/

#define MOTOR_T_OFFSET 0   // Speed offset for tail motor.
#define MOTOR_R_OFFSET 0   // Speed offset for right motor.
#define MOTOR_L_OFFSET 0   // Speed offset for left motor.
#define MOTOR_T_SCALE  1.18   // Scale speed of tail motor.
#define MOTOR_R_SCALE  1   // Scale speed of right motor.
#define MOTOR_L_SCALE  1   // Scale speed of left motor.
#define TAIL_SERVO_DEFAULT_POSITION 60
#define TAIL_SERVO_SCALE 70   // Scale tail servo rotation.

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

// From http://www.utopiamechanicus.com/399/low-memory-serial-print/
//void StreamPrint_progmem(Print &out, PGM_P format, ...) {
//    // program memory version of printf - copy of format string and result share a buffer
//    // so as to avoid too much memory use
//    char formatString[256], *ptr;
//    strncpy_P(formatString, format, sizeof(formatString));   // copy in from program mem
//    // null terminate - leave last char since we might need it in worst case for result's \0
//    formatString[sizeof(formatString)-2] = '\0';
//    ptr=&formatString[strlen(formatString)+1]; // our result buffer...
//    va_list args;
//    va_start(args, format);
//    vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args);
//    va_end(args);
//    formatString[sizeof(formatString)-1] = '\0';
//    out.print(ptr);
//}
//
//#define serPrint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)
//#define serPrint(format, ...) if (teleCount == TELEMETRY_LOOP_INTERVAL) StreamPrint_progmem(Serial, PSTR(format), ##__VA_ARGS__)
//#define Streamprint(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)

#define sp Serial.print
#define spln Serial.println
#define sw Serial.write

//#define serPrint Serial.print

#endif // GLOBALS_H

