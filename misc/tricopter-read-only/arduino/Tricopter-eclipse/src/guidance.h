/*
 * guidance.h
 *
 *  Created on: Jun 5, 2010
 *      Author: ben
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#define GUIDANCE_INPUT_TIMEOUT 1000000
#define GUIDANCE_YAW_SCALE 20
#define GUIDANCE_LIFT_SCALE .0025
#define GUIDANCE_LIFT_PWR 3

enum GuidanceMode {OFF=0, ON};

struct Guidance {
  float lift;
  int yaw;
  int pitch;
  int roll;

  long lastNewInput;
  GuidanceMode mode;
  long runtime;
};

void initGuidance(struct Guidance * guid);
void updateGuidance(struct Guidance *guid, struct Input *input);

#endif /* GUIDANCE_H_ */
