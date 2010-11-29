#include <WProgram.h>
#include <Servo.h>
#include <Messenger.h>
#include <Wire.h>

/* Misc Constants ***************************************/
#define FRAME_LENGTH 20000 //20 millisecond frames
#define LEDPIN 13

//Flight Modes
#define GROUND_MODE 0
#define LIFTOFF_MODE 1

/* Input Constants **************************************/
#define BAUD_RATE 57600

//Input Message Order Constants
#define INPUT_NUM_WORDS 7
#define INPUT_SEQUENCE 0
#define INPUT_LIFT 1
#define INPUT_YAW 2
#define INPUT_PITCH 3
#define INPUT_ROLL 4
#define INPUT_BUTTON 5
#define INPUT_CHECKSUM 6

// Joystick Button Constants
#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_X 2
#define BUTTON_Y 3
#define BUTTON_LB 4
#define BUTTON_RB 5
#define BUTTON_BACK 6
#define BUTTON_START 7
#define BUTTON_L_JOY 8
#define BUTTON_R_JOY 9

//Input Failure Codes
#define INPUT_NO_FAIL 0
#define INPUT_NUM_WORDS_FAIL 1
#define INPUT_CHECKSUM_FAIL 2
#define INPUT_SEQUENCE_FAIL 3

/* Sensor Constants *************************************/
#define NUM_AXES 3
#define ROLL_AXIS 0
#define PITCH_AXIS 1
#define YAW_AXIS 2

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// Gyro Analog ADC PIN constants
#define ROLL_GYRO_PORT 0
#define PITCH_GYRO_PORT 1
#define YAW_GYRO_PORT 2
const int GYRO_PORTS[NUM_AXES] = {
  ROLL_GYRO_PORT, PITCH_GYRO_PORT, YAW_GYRO_PORT};

// Accel Register Constants
#define X_ACCEL_ADDR 0x2A
#define Y_ACCEL_ADDR 0x28
#define Z_ACCEL_ADDR 0x2C
const int ACCEL_ADDRS[NUM_AXES] = {
  X_ACCEL_ADDR, Y_ACCEL_ADDR, Z_ACCEL_ADDR};

/* Axis Constants **************************************/
#define LIFT_IN_MAX 999
#define LIFT_OUT_MAX 999
#define ROLL_PITCH_IN_MAX 999 
#define ROLL_PITCH_OUT_MAX 50
#define YAW_IN_MAX 999
#define YAW_OUT_MAX 50
#define YAW_CORRECTION 1.05

/* Motor Constants **************************************/
//Motor Command Bounds
#define MOTOR_MIN_COMMAND 1000
#define MOTOR_MID_COMMAND 1500
#define MOTOR_MAX_COMMAND 2000
const int MOTOR_MIN[] = {
  1060, 1060, 1060, 1060, 1060, 1060};
const int MOTOR_MAX[] = {
  1700, 1700, 1700, 1700, 1700, 1700};

//Motor Array Positions
#define NUM_MOTORS 6
#define RIGHT_TOP 0
#define RIGHT_BOTTOM 1
#define LEFT_TOP 2
#define LEFT_BOTTOM 3
#define BACK_TOP 4
#define BACK_BOTTOM 5

//Motor Output Ports
#define RIGHT_TOP_PORT 2
#define RIGHT_BOTTOM_PORT 3
#define LEFT_TOP_PORT 6
#define LEFT_BOTTOM_PORT 7
#define BACK_TOP_PORT 8
#define BACK_BOTTOM_PORT 9
const int MOTOR_PORTS[NUM_MOTORS] = 
{
  RIGHT_TOP_PORT, RIGHT_BOTTOM_PORT, LEFT_TOP_PORT, LEFT_BOTTOM_PORT, BACK_TOP_PORT, BACK_BOTTOM_PORT};


/* Variables ********************************************/
int flightMode;

/* Input Variables *************************************/
//inputMessage is parsed into inputArray
Messenger inputMessage = Messenger();
int inputArray[INPUT_NUM_WORDS];
long inputNumFails;
int inputFailCode;
boolean newMessage;

/* Sensor Variables *************************************/
//Calibrated zero inputs
int gyroZero[NUM_AXES];
int accelZero[NUM_AXES];
//Raw readings from sensors
int gyroRaw[NUM_AXES];
int accelRaw[NUM_AXES];
//Smoothed data from sensors
float gyroSmooth[NUM_AXES];
float accelSmooth[NUM_AXES];
//Attitude of aircraft obtained from complementary filter for pitch and roll, and simple integration for yaw
float flightAngle[NUM_AXES];
float resultant;

/* Axis Variables ****************************************/
int cmdLift;
int cmdYaw;
int cmdPitch;
int cmdRoll;
int cmdButtons;

/* Control variables */
struct PIDdata {
  float P_GAIN, I_GAIN, D_GAIN;
  float pTerm, iTerm, dTerm;
  float lastPosition;
  float integratedError;
} 
PID[2];

float windupGuard = 1000;

/* Motor Variables ****************************************/
Servo motors[NUM_MOTORS];  
int motorCommands[NUM_MOTORS];


/* Telemetry Variables ****************************************/
//Telemetry Sequence incremented by 1 each time tlm is sent.
int tlmSequence = 0;
int tlmChecksum = 0;
int sendTlm = 0;


/* Timing Variables ****************************************/
long previousTime = 0;
long currentTime = 0;
long deltaTime = 0;
long lastFrameTime = 0;
long dutyCycle = 0;
float dt = 0;

// Initialize everything
void setup() {
  pinMode (LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  initInput();
  initSensors();
  initAxis();
  initMotors();
  initTelemetry();
  flightMode = GROUND_MODE;
  previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
}


void loop() {

  /* Process input from XBee 
   * This happens asynchronously from everything else.
   * processInput() will update all the input commands (or configuration)
   * whenever a new message is received.
   */
  while (Serial.available()) {
    inputMessage.process(Serial.read());
    if (inputMessage.available()) {
      processInput();
    }
  }

  /* Keep track of time */
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;
  dt = deltaTime * .000001;
  lastFrameTime = currentTime;

  /* Update input processor with current time to detect connection timouts */

  /*********************************************************************************/
  //Update sensor data and IMU stuff every loop, 
  //Sensors must be read as fast as possible for best results.

  //Read raw sensors and subtract from zero point to get raw rate
  //Roll and pitch must be negated since the sensor is "backwards" to the reference frame
  for (int i = 0; i < NUM_AXES; i++) {
    gyroRaw[i] = analogRead(GYRO_PORTS[i]) - gyroZero[i];
    accelRaw[i] = i2cReadAccel(ACCEL_ADDRS[i]) - accelZero[i];
  }
  gyroRaw[ROLL_AXIS] = -gyroRaw[ROLL_AXIS];
  gyroRaw[PITCH_AXIS] = -gyroRaw[PITCH_AXIS];
  //accelRaw[Y_AXIS] = -accelRaw[Y_AXIS];
  accelRaw[Z_AXIS] = -accelRaw[Z_AXIS];

  //Run raw readings through low pass filter
  for (int i = 0; i < NUM_AXES; i++) {
    gyroSmooth[i] = gyroSmooth[i] * .75 + gyroRaw[i] * .25;
    accelSmooth[i] = accelSmooth[i] * .75 + accelRaw[i] * .25;
  }

  //Compute resultant
  resultant = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    resultant += accelSmooth[i]*accelSmooth[i];
  }
  resultant = sqrt(resultant)*10/1024;

  //Determine flight angle
  flightAngle[PITCH_AXIS] = atan2(-accelSmooth[Y_AXIS], accelSmooth[Z_AXIS])*(180/PI)+180;
  flightAngle[ROLL_AXIS] = atan2(-accelSmooth[X_AXIS], accelSmooth[Z_AXIS])*(180/PI)+180;
  for (int i = 0; i < NUM_AXES; i++) {
    if (flightAngle[i] > 180.0) {
      flightAngle[i] -= 360.0;
    }
  }

  //Simple integration to determine flight angle (times 10)
  //flightAngle[ROLL_AXIS] += gyroSmooth[ROLL_AXIS] * .3538 * dt * 10;
  //flightAngle[PITCH_AXIS] += gyroSmooth[PITCH_AXIS] * .3538 * dt * 10;
  //flightAngle[YAW_AXIS] += gyroSmooth[YAW_AXIS] * .9765625 * dt * 10;
  /*********************************************************************************/


  cmdLift = inputArray[INPUT_LIFT] + 1000;//map(inputArray[INPUT_LIFT], LIFT_IN_MIN, LIFT_MAX, LIFT_MIN, LIFT_MAX);
  cmdYaw = inputArray[INPUT_YAW] / 20; //map(inputArray[INPUT_YAW], -YAW_IN_MAX, YAW_IN_MAX, -YAW_OUT_MAX, YAW_OUT_MAX);
  //cmdPitch = map(inputArray[INPUT_PITCH], -ROLL_PITCH_IN_MAX, ROLL_PITCH_IN_MAX, -ROLL_PITCH_OUT_MAX, ROLL_PITCH_OUT_MAX);
  //cmdRoll = -map(inputArray[INPUT_ROLL],  -ROLL_PITCH_IN_MAX, ROLL_PITCH_IN_MAX, -ROLL_PITCH_OUT_MAX, ROLL_PITCH_OUT_MAX);

  cmdPitch = updatePID(0, flightAngle[PITCH_AXIS], &PID[PITCH_AXIS]);
  cmdRoll = updatePID(0, flightAngle[ROLL_AXIS], &PID[ROLL_AXIS]);

  if (inputButtonStatus(BUTTON_LB)) {
    for (int i = 0; i <= PITCH_AXIS; i++) {
      PID[i].P += 0.025;
    }
  }
  if (inputButtonStatus(BUTTON_RB)) {
    for (int i = 0; i <= PITCH_AXIS; i++) {
      PID[i].P -= 0.025;
    }
  }

  if (inputButtonStatus(BUTTON_Y)) {
    for (int i = 0; i <= PITCH_AXIS; i++) {
      PID[i].I += 0.025;
    }
  }
  if (inputButtonStatus(BUTTON_B)) {
    for (int i = 0; i <= PITCH_AXIS; i++) {
      PID[i].I -= 0.025;
    }
  }

  // }


  /* Control Loop
   * Run once every FRAME_LENGTH since motors can't be commanded as fast as sensors can be read.
   */
  //  if (currentTime - lastFrameTime > FRAME_LENGTH) {
  //    lastFrameTime = currentTime;

  //Do feedback control
  switch (flightMode) {

  case GROUND_MODE:
    commandAllMotors(MOTOR_MIN_COMMAND);

    if (inputButtonStatus(BUTTON_START)) {
      flightMode = LIFTOFF_MODE;
    }
    if (inputButtonStatus(BUTTON_X) == 1) {
      initSensors();
    }
    break;


  case LIFTOFF_MODE:

    motorCommands[RIGHT_TOP] = int(cmdLift + cmdYaw + cmdPitch/3.0 + cmdRoll/1.73205);
    motorCommands[RIGHT_BOTTOM] = int(cmdLift - cmdYaw + cmdPitch/3.0 + cmdRoll/1.73205);
    motorCommands[LEFT_TOP] = int(cmdLift + cmdYaw - cmdPitch/3.0 - cmdRoll/1.73205);
    motorCommands[LEFT_BOTTOM] = int(cmdLift - cmdYaw - cmdPitch/3.0 - cmdRoll/1.73205);
    motorCommands[BACK_TOP] = int(cmdLift + cmdYaw + 2.0*cmdPitch/3.0);
    motorCommands[BACK_BOTTOM] = int(cmdLift - cmdYaw + 2.0*cmdPitch/3.0);

    commandMotors();

    //Motor Kill Switch
    if (inputButtonStatus(BUTTON_BACK)) {
      flightMode = GROUND_MODE;
    } 
    break;
  } //Switch

  dutyCycle = micros() - lastFrameTime;

  //Send Telemetry every 10 frames
  if (sendTlm++ % 100 == 0)
    sendTelemetry();
  //  }
} 









