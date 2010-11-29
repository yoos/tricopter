#include <WProgram.h>
#include <Servo.h>
#include <Messenger.h>
#include <Wire.h>

#include "constants.h"
#include "pid.h"
#include "sensors.h"
#include "input.h"
#include "nav.h"
#include "control.h"
#include "motors.h"
#include "utils.h"

/* Input Variables *************************************/
//inputMessage is parsed into inputArray
Messenger inputMessage = Messenger();
struct Input input;

/* Guidance */
struct Guidance guid;

/* Sensor Variables *************************************/
struct Accel accels[NUM_AXES];
struct Gyro gyros[NUM_AXES];

//Attitude of aircraft obtained from complementary filter for pitch and roll, and simple integration for yaw
struct Navigator nav;

// Controller
struct Controller controller;

/* Motor Variables ****************************************/
Servo motors[NUM_MOTORS];

/* Telemetry Variables ****************************************/
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
  Serial.begin(BAUD_RATE);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  initInput(&input);
  initGuidance(&guid);
  //initSensors(accels, gyros);
  //  initControl();
  //initMotors(motors);
  //  initTelemetry();
  //flightMode = GROUND_MODE;
  //previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
  DEBUG("Done with init","!");
}

void loop() {
  //  timerStartInput(&timer);

  /* Process input from XBee 
   * This happens asynchronously from everything else.
   * processInput() will update all the input commands (or configuration)
   * whenever a new message is received.
   */
  while (Serial.available()) {
    inputMessage.process(Serial.read());
    if (inputMessage.available()) {
      processInput(&inputMessage, &input);
    }
  }

  currentTime = micros();
  if (currentTime - lastFrameTime > FRAME_LENGTH) {
    lastFrameTime = currentTime;

    updateGuidance(&guid, &input);
  }


  /*
   * Read Raw Sensor Data
   * Roll and pitch must be negated since the sensor is
   * "backwards" to the reference frame
   */
  //    timerStartSensors(&timer);
  //updateGyros(gyros);
  //gyros[ROLL_AXIS].value = -gyros[ROLL_AXIS].value;
  //gyros[PITCH_AXIS].value = -gyros[PITCH_AXIS].value;
  //updateAccels(accels);
  //accels[Z_AXIS].value = -accels[Z_AXIS].value;
  //    timerEndSensors(&(timer));

  /*
   * Update the Navigation Solution with the new sensor data
   */
  //    timerStartNav(&(timer));
  //updateNav(&nav, accels, gyros);
  //    timerEndNav(&(timer));

  /*
   * Update the feedback loop.
   */
  //    timerStartControls(&timer);
  //updateController(&guid, &nav, &controller);
  //    timerEndControls(&timer);

  /*
   * Update the motor commands.
   */
  //updateMotors(motors, &controller);

  //dutyCycle = micros() - lastFrameTime;

  //Send Telemetry every 10 frames
  //if (sendTlm++ % 100 == 0)
  //sendTelemetry();
  //  }
  //  }
}

extern "C" void __cxa_pure_virtual() {
  cli();
  for (;;)
    ;
}
