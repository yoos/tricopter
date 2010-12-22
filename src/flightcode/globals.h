#ifndef GLOBALS_H
#define GLOBALS_H


// Configurables

#define DEBUG

#define IMU_SAMPLE_INTERVAL 10   // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1

#define DOGBONE char(255)
#define DOGLIFE 500   // Watchdog life in milliseconds

#define MOTOR_TEST_PIN 13
#define MTP2 12


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













#endif // GLOBALS_H

