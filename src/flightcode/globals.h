#ifndef GLOBALS_H
#define GLOBALS_H


// Configurables

#define DEBUG

#define BAUDRATE 9600
#define IMU_SAMPLE_INTERVAL 25   // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1

#define SERHEAD char(255)
#define DOGBONE char(254)
#define DOGLIFE 200   // Watchdog life in milliseconds


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

#define MOTOR_L 10 // Left motor PWM
#define MOTOR_R 11 // Right motor PWM
#define MOTOR_T 12 // Tail motor PWM

#define SERVO_T 13 // Tail servo PWM for yaw control


void zeroStr(char *sStr) {
    for (int i=0; i<128; i++) {
        sStr[i] = 0;
    }
}










#endif // GLOBALS_H

