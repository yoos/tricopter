/*
 * input.h
 *
 *  Created on: Mar 23, 2010
 *      Author: ben
 */


#ifndef INPUT_H_
#define INPUT_H_

#include <Messenger.h>

/* Input Constants **************************************/
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

//Input Message Order Constants
/*
#define INPUT_NUM_WORDS 7
#define INPUT_SEQUENCE 0
#define INPUT_LIFT 1
#define INPUT_YAW 2
#define INPUT_PITCH 3
#define INPUT_ROLL 4
#define INPUT_BUTTON 5
#define INPUT_CHECKSUM 6

//Input Failure Codes
#define INPUT_NO_FAIL 0
#define INPUT_NUM_WORDS_FAIL 1
#define INPUT_CHECKSUM_FAIL 2
#define INPUT_SEQUENCE_FAIL 3
*/

//Globals
struct Input {
  int newInput;
  int sequence;
  int leftVerticalAxis;
  int leftHorizontalAxis;
  int rightVerticalAxis;
  int rightHorizontalAxis;
  int leftTrigger;
  int rightTrigger;
  int buttons;
  int lastButtons;
};



void initInput(struct Input *input);
void processInput(Messenger *inputMessage, struct Input *input);
int inputButtonStatus(struct Input *input, int button);
int inputButtonPressed(struct Input *input, int button);

#endif /* INPUT_H_ */
