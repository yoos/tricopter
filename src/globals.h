#ifndef GLOBALS_H
#define GLOBALS_H

// #define DEBUG

/*****************************************************************************
 * Variables
 *****************************************************************************/

int armCount;   // Arm status counter.
int loopCount;   // Count system loops.
float pwmOut[4];
char commStr[250];   // String to be sent out to base.
float commandPitch;   // Pitch command to be processed through PID.
float commandRoll;   // Roll command to be processed through PID.
float gyroDCM[3][3];   // Current position DCM calculated by IMU.
float targetRot[3], currentRot[3], pidRot[3];
float gVal[3];   // [-2000,2000] deg/s mapped to [-1,1]


// ============================================================================
// PID
//     Match the number of structs to the number of PID value defines.
// ============================================================================
struct PIDdata {
    float P, I, D;
    float lastValue;
    float integral;
} PID[4];

//#define PID_MOTOR_T 0
//#define PID_MOTOR_R 1
//#define PID_MOTOR_L 2
//#define PID_SERVO_T 3

#define PID_ROT_X 0
#define PID_ROT_Y 1
#define PID_ROT_Z 2

#define XY_P_GAIN 30.0   // 35.0
#define XY_I_GAIN 20.0   // 50.0
#define XY_D_GAIN -10.0   // -9.0

#define Z_P_GAIN 100.0
#define Z_I_GAIN 0.0
#define Z_D_GAIN 0.0

/*****************************************************************************
 * Serial: everything that has to do with TX/RX.
 *****************************************************************************/

// ============================================================================
// SERIAL IN
// ============================================================================
#define BAUDRATE 57600   // Fiddle with this. The XBee sometimes seems to have trouble with high baudrates like 57600.
#define SERHEAD    255   // Serial header byte. Pilot interprets the four bytes following this header byte as motor commands.
#define PACKETSIZE 6     // Each packet contains (excluding header) X, Y, Twist, Z, and two bytes for button values.
#define INPUT_MIN  0     // Minimum integer input value from joystick.
#define INPUT_MAX  250   // Maximum integer input value from joystick.
#define SX 0   // Serial byte location for joystick X axis.
#define SY 1   // Serial byte location for joystick Y axis.
#define ST 2   // Serial byte location for joystick T (twist) axis.
#define SZ 3   // Serial byte location for joystick Z axis.
#define SB1 4   // Serial byte location for joystick buttons (0 to 7).
#define SB2 5   // Serial byte location for joystick buttons (8 to 15).

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

#define MASTER_DT              10   // 10 ms interval = 100 Hz master loop.
#define CONTROL_LOOP_INTERVAL   2   // 1/2 master = 50 Hz. NOTE: This frequency should be HIGHER than comm.py's dataSend frequency!
#define TELEMETRY_LOOP_INTERVAL 5   // 1/5 master = 20 Hz.
#define DOGLIFE 300   // Watchdog life in milliseconds.

//#define DCM_COEFF 90   // Scale current-to-target DCM difference.
//#define GYRO_COEFF 15   // Try to stabilize craft.
//#define ACCEL_COEFF 90   // TEST: Try to stabilize craft.

// Throttle stuff. Minimum signal is 750 ms. Maximum signal is 2200 ms. Hover
// is around 1200 ms.
#define TMIN   750   // Minimum throttle signal in ms. (Absolute minimum is 750.)
#define THOVER 1200   // Hover throttle signal in ms.
#define TMAX   1500   // Maximum throttle signal in ms. (Absolute maximum is 2200.)

#define TIME_TO_ARM 2000   // This divided by MASTER_DT determines how long it takes to arm the system.
#define MOTOR_ARM_THRESHOLD 30   // This is added to TMIN to determine whether or not to arm the system.

#define MOTOR_T 0   // Tail motor array index.
#define MOTOR_R 1   // Right motor array index.
#define MOTOR_L 2   // Left motor array index.
#define SERVO_T 3   // Tail servo array index.

// ============================================================================
// Buttons
// ============================================================================
#define BUTTON_UNDEFINED            0
#define BUTTON_RESET_YAW            1
#define BUTTON_ZERO_INTEGRAL        2
#define BUTTON_DECREASE_TRIM        3
#define BUTTON_UNDEFINED            4
#define BUTTON_INCREASE_TRIM        5
#define BUTTON_DECREASE_XY_P_GAIN   6
#define BUTTON_INCREASE_XY_P_GAIN   7
#define BUTTON_DECREASE_XY_I_GAIN   8
#define BUTTON_INCREASE_XY_I_GAIN   9
#define BUTTON_DECREASE_XY_D_GAIN   10
#define BUTTON_INCREASE_XY_D_GAIN   11

/*****************************************************************************
 * Hardware configuration: any parameter that is changed so infrequently that
 * it may as well be hard-coded.
 *****************************************************************************/

#define MOTOR_T_OFFSET 0   // Speed offset for tail motor.
#define MOTOR_R_OFFSET 0   // Speed offset for right motor.
#define MOTOR_L_OFFSET 0   // Speed offset for left motor.
#define MOTOR_T_SCALE  1   // Scale speed of tail motor.
#define MOTOR_R_SCALE  1   // Scale speed of right motor.
#define MOTOR_L_SCALE  1   // Scale speed of left motor.
#define TAIL_SERVO_DEFAULT_POSITION 1250
#define TAIL_SERVO_SCALE 1   // Scale tail servo rotation.
#define Z_ROT_SPEED 1   // Scale how much joystick twist input affects target Z rotation. A value of 1 here means a maximum Z rotation speed is 1 rad/s.

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

