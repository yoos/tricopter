#ifndef CONFIG_H
#define CONFIG_H


// Configurables

#define IMU_SAMPLE_INTERVAL 10 // In milliseconds
#define GYRO_VREF 1
#define ACCEL_VREF 1



// Flight modes

#define OFF 0
#define IDLE 1
#define HOVER 2
#define ACRO 3
#define AUTO 4
#define AUTO_HOVER 5

// Digital pins

#define MOTOR_L 10 // Left motor
#define MOTOR_R 11 // Right motor
#define MOTOR_T 12 // Tail motor

#define SERVO_T 13 // Tail servo for yaw control




// Analog pins

#define GYRO_X 0
#define GYRO_Y 1
#define GYRO_Z 2
#define ACCEL_X 3
#define ACCEL_Y 4
#define ACCEL_Z 5















#endif

