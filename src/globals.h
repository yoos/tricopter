/*! \file globals.h
 *  \author Soo-Hyun Yoo
 *  \brief Various global definitions and variables.
 *
 *  Details.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

// #define DEBUG

// ============================================================================
// Variables
// ============================================================================
int armCount;   // Arm status counter.
int loopCount;   // Count system loops.
int16_t pwmShift[4], pwmOut[4];   // 10 bit PWM output duty cycle.
float bodyDCM[3][3];   // Current body orientation calculated by IMU.
float targetAngPos[3], targetAngRate[3], pidAngPos[3], pidAngRate[3], currentAngPos[3];
float gVec[3];   // This used to be part of ITG3200, but is now global so the PID controller can have direct access to the gyro measurements. This is a hack, and I am a bad programmer.

// ============================================================================
// PID
//     Match the number of structs to the number of PID value defines.
// ============================================================================
struct PIDdata {
    uint8_t id;
    float deltaPIDTime;
    float P, I, D;
    float lastValue;
    float integral;
    float lastDerivative;
} PID[10];

#define PID_ANG_POS_X  0
#define PID_ANG_POS_Y  1
#define PID_ANG_POS_Z  2
#define PID_ANG_RATE_X 3
#define PID_ANG_RATE_Y 4
#define PID_ANG_RATE_Z 5

#define XY_ANG_POS_P_GAIN  0.0
#define XY_ANG_POS_I_GAIN  0.0
#define XY_ANG_POS_D_GAIN -0.0
#define XY_ANG_RATE_P_GAIN  5.0
#define XY_ANG_RATE_I_GAIN  0.0
#define XY_ANG_RATE_D_GAIN -0.0

#define Z_ANG_POS_P_GAIN 0.0
#define Z_ANG_POS_I_GAIN 0.0
#define Z_ANG_POS_D_GAIN 0.0
#define Z_ANG_RATE_P_GAIN 0.0
#define Z_ANG_RATE_I_GAIN 0.0
#define Z_ANG_RATE_D_GAIN 0.0

#define TARGET_ANGLE_CAP PI/12   // Cap difference between target and current rotation vectors.
#define TARGET_RATE_CAP 2*PI   // Cap maximum angular velocity rate.
#define XY_D_TERM_CAP 0.6   // Cap maximum angular acceleration rate.

#define THROTTLE_LOCK_DIFF_UP   70   // "Lock" throttle input. See pilot.h.
#define THROTTLE_LOCK_DIFF_DOWN 120   // "Lock" throttle input. See pilot.h.


// ============================================================================
// SERIAL IN
// ============================================================================
#define BAUDRATE 38400   // Do NOT use 57600 or 115200 here, because the error rate is too high!
#define SERHEAD    255   // Serial header byte. Pilot interprets the four bytes following this header byte as motor commands.
#define SER_PACKET_LEN 6     // Each packet contains (excluding header) X, Y, Twist, Z, and two bytes for button values.
#define SER_READ_BUF_LEN 100   // Number of bytes of serial data in buffer (used by Pilot).
#define SER_READ_CHUNK_LEN 2   // Number of bytes to read off serial bus every loop.

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
#define SER_WRITE_BUF_LEN 150   // Number of bytes of serial data in TX buffer.
#define SER_WRITE_CHUNK_LEN 15   // Number of bytes to send per loop.

#define SEND_ARM_STATUS
//#define SEND_TARGET_ROTATION
#define SEND_MOTOR_VALUES
#define SEND_DCM
#define SEND_PID_DATA

#define DCM_SER_TAG 0xfb
#define ROT_SER_TAG 0xfc
#define MOT_SER_TAG 0xfd
#define PID_SER_TAG 0xfe
#define FIELD_SER_TAG 0xff


// ============================================================================
// Software configuration: any parameter that is purely code-related or is
// relatively frequently changed.
// ============================================================================
#define MASTER_DT            6000   // 6000 us interval = 166 Hz master loop.
#define CONTROL_LOOP_INTERVAL   1   // 1x master = 166 Hz.
#define ACC_READ_INTERVAL       5   // Read accelerometer every 5th loop.
#define COMM_LOOP_INTERVAL      5   // 1/5 master = 33 Hz.
#define DOGLIFE 600   // Watchdog life in milliseconds.

// Throttle stuff. Minimum signal is 750 us. Maximum signal is 2200 us. Hover
// is around 1200 us.
#define TMIN   432   // Minimum throttle PWM duty cycle. At 400 kHz, 2500 * 432/1023 = 1055 us. Although simonk's firmware should register 1060 us as the minimum throttle, one of my ESCs will not arm until it is this low.
#define THOVER 480   // Hover throttle PWM duty cycle
#define TMAX   600   // Maximum throttle PWM duty cycle (at 400 kHz, 2500 * 761/1023 = 1860 us).

#define SERVO_US_ZERO 1430   // Servo "zero" position (i.e., level to chassis).
#define SERVO_US_NEUTRAL 1370   // Servo neutral position (i.e., net Z torque = 0).
#define SERVO_US_PER_RAD 500   // Microseconds per radian of servo rotation.

#define TIME_TO_ARM 2000000   // This divided by MASTER_DT determines how long it takes to arm the system.
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
#define BUTTON_ACRO_MODE            2
#define BUTTON_DECREASE_TRIM        3
#define BUTTON_ZERO_INTEGRAL        4
#define BUTTON_INCREASE_TRIM        5
#define BUTTON_DECREASE_XY_P_GAIN   6
#define BUTTON_INCREASE_XY_P_GAIN   7
#define BUTTON_DECREASE_XY_I_GAIN   8
#define BUTTON_INCREASE_XY_I_GAIN   9
#define BUTTON_DECREASE_XY_D_GAIN   10
#define BUTTON_INCREASE_XY_D_GAIN   11

// ============================================================================
// Hardware configuration: any parameter that is changed so infrequently that
// it may as well be hard-coded.
// ============================================================================

#define MOTOR_T_OFFSET 0   // Speed offset for tail motor.
#define MOTOR_R_OFFSET 0   // Speed offset for right motor.
#define MOTOR_L_OFFSET 0   // Speed offset for left motor.
#define MOTOR_T_SCALE  1   // Scale speed of tail motor.
#define MOTOR_R_SCALE  1   // Scale speed of right motor.
#define MOTOR_L_SCALE  1   // Scale speed of left motor.
#define TAIL_SERVO_SCALE 1   // Scale tail servo rotation.
#define Z_ROT_SPEED 1   // Scale how much joystick twist input affects target Z rotation. A value of 1 here means a maximum Z rotation speed is 1 rad/s.

// Calibration values for accelerometer.
#define ACCEL_X_OFFSET -0.033
#define ACCEL_Y_OFFSET -0.011
#define ACCEL_Z_OFFSET -0.999

// Calibration values for magnetometer. These are what the magnetometer axes
// see as "zero".   TODO: This is not entirely accurate. Need to figure out how
// magnetometers actually work!
#define MAG_X_MIN -314
#define MAG_X_MAX 320
#define MAG_Y_MIN -316
#define MAG_Y_MAX 317
#define MAG_Z_MIN -427
#define MAG_Z_MAX 165

#define PMT 5   // Tail motor pin.
#define PMR 2   // Right motor pin.
#define PML 3   // Left motor pin.
#define PST 4   // Tail servo pin.


// ============================================================================
// Flight modes: not yet implemented.
// ============================================================================

#define OFF 0
#define IDLE 1
#define HOVER 2
#define ACRO 3
#define AUTO 4
#define AUTO_HOVER 5


// ============================================================================
// Constants
// ============================================================================

#define PI 3.141592653589793238462643383279502884197f


// ============================================================================
// Functions
// ============================================================================

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

