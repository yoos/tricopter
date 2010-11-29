#include <WProgram.h>
#include <Messenger.h>
#include "input.h"
#include "utils.h"
#include "telemetry.h"

void processJoystick(Messenger *inputMessage, struct Input *input);
void sendInputTlm(struct Input *input);

void initInput(struct Input *input) {
  input->sequence = -1;
  input->newInput = false;
}

void processInput(Messenger *inputMessage, struct Input *input) {
  char messageType;
  //First word is type in input message
  messageType = inputMessage->readChar();
  switch (messageType) {
    case 'J':  //Joystick Message
      //DEBUG("Got Joystick Message", " J");
      processJoystick(inputMessage, input);
      break;
    default:
      DEBUG("Received unknown inputMessage type", messageType);
  }
}

void processJoystick(Messenger *inputMessage, struct Input *input) {
  input->sequence = inputMessage->readInt();
  input->leftVerticalAxis = inputMessage->readInt();
  input->leftHorizontalAxis = inputMessage->readInt();
  input->rightVerticalAxis = inputMessage->readInt();
  input->rightHorizontalAxis = inputMessage->readInt();
  input->leftTrigger = inputMessage->readInt();
  input->rightTrigger = inputMessage->readInt();
  input->lastButtons = input->buttons;
  input->buttons = inputMessage->readInt();
  input->newInput = true;
  //sendInputTlm(input);
}

void sendInputTlm(struct Input *input) {
  TLM('I');
  TLM(input->sequence);
  TLM(input->leftVerticalAxis);
  TLM(input->leftHorizontalAxis);
  TLM(input->rightVerticalAxis);
  TLM(input->rightHorizontalAxis );
  TLM(input->leftTrigger);
  TLM(input->rightTrigger);
  TLM(input->lastButtons);
  TLM(input->buttons);
  TLM( input->newInput);
  TLM_CLOSE();
}

//Get the status of a button number from the Command Message
int inputButtonStatus(struct Input *input, int button) {
  return bitRead(input->buttons, button);
}

int inputButtonPressed(struct Input *input, int button) {
  return bitRead(input->buttons, button) && !bitRead(input->lastButtons, button);
}
/*


  int inputNext[INPUT_NUM_WORDS];
  int i;
  input->failCode = INPUT_NO_FAIL;

  //Parse message
  //Check for correct number of words
  for (i = 0; i < INPUT_NUM_WORDS; i++) {
    inputNext[i] = 0;

    if (inputMessage->available()) {
     inputNext[i] = inputMessage->readInt();
    }

    else {
      input->failCode = INPUT_NUM_WORDS_FAIL;
      input->numFails++;
      return;
    }

  }

  //Check checksum
  if (sumArray(inputNext, 1, INPUT_NUM_WORDS-1) != inputNext[INPUT_CHECKSUM]) {
    input->failCode = INPUT_CHECKSUM_FAIL;
    input->numFails++;
    return;
  }

  //Check sequence
  if (inputNext[INPUT_SEQUENCE] != input->sequence + 1) {
    input->numFails++;
    // Count it as a fail but still process the input
  }

  input->lift = constrain(inputNext[INPUT_LIFT], 0, 999);
  input->yaw = inputNext[INPUT_YAW];
  input->pitch = inputNext[INPUT_PITCH];
  input->roll = inputNext[INPUT_ROLL];
  input->buttons = inputNext[INPUT_BUTTON];
  input->sequence = inputNext[INPUT_SEQUENCE];
  input->newInput = true;
}
*/

