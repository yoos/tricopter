/*
 * guidance.cpp
 *
 *  Created on: Jun 5, 2010
 *      Author: ben
 */
#include <WProgram.h>
#include <stdio.h>
#include "guidance.h"
#include "input.h"
#include "utils.h"
#include "telemetry.h"

void zeroGuidance(struct Guidance *guid);
void sendGuidanceTlm(struct Guidance *guid);

void initGuidance(struct Guidance * guid) {
  zeroGuidance(guid);
  guid->lastNewInput = -1;
  guid->mode = OFF;
}

void updateGuidance(struct Guidance *guid, struct Input *input) {
  long startTime = micros();
  if (input->newInput == true) {
    input->newInput = false;
    guid->lastNewInput = micros();
    switch (guid->mode) {
      case OFF:
        zeroGuidance(guid);
        if (inputButtonPressed(input, BUTTON_START)) {
          guid->mode = ON;
        }
        break;
      case ON:
        if (inputButtonPressed(input, BUTTON_BACK)) {
          // Back button turns everything off
          zeroGuidance(guid);
          guid->mode = OFF;
        } else {
          // This is where everything really happens
          // Always command level flight (for now)
          // Yaw is direct
          // Lift is accumulative
          guid->pitch = 0;
          guid->roll = 0;
          guid->yaw = ((input->leftTrigger - input->rightTrigger) / 2)
              / GUIDANCE_YAW_SCALE;
          guid->lift += pow(
              (input->leftVerticalAxis * GUIDANCE_LIFT_SCALE),
              GUIDANCE_LIFT_PWR);
          guid->lift = limitFloatRange(guid->lift, 0.0, 1000.0);
        }
        break;
    }
  } else {
    if ((micros() - guid->lastNewInput) > GUIDANCE_INPUT_TIMEOUT) {
      // Turn everything off if timeout reached
      zeroGuidance(guid);
      guid->mode = OFF;
      DEBUG("Timeout waiting for input message","BLA");
    }
    // If waiting for next input, and timeout hasn't been reached yet, do nothing
  }
  guid->runtime = micros()-startTime;
  sendGuidanceTlm(guid);
}

void zeroGuidance(struct Guidance *guid) {
  guid->lift = 0;
  guid->yaw = 0;
  guid->pitch = 0;
  guid->roll = 0;
}

void sendGuidanceTlm(struct Guidance *guid) {
  TLM('G');
  TLM(int(guid->lift));
  TLM(guid->yaw);
  TLM(guid->pitch);
  TLM(guid->roll);
  TLM(guid->lastNewInput);
  TLM(guid->mode);
  TLM(guid->runtime);
  TLM_CLOSE();
}
