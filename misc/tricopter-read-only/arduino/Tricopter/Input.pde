void initInput() {
  Serial.begin(BAUD_RATE);
  inputNumFails = 0;
  inputFailCode = INPUT_NO_FAIL;
  newMessage = false;
}

void processInput() {
  int inputNext[INPUT_NUM_WORDS];
  inputFailCode = INPUT_NO_FAIL;

  //Parse message
  //Check for correct number of words
  for (int i = 0; i < INPUT_NUM_WORDS; i++) {
    if (inputMessage.available()) {
      inputNext[i] = inputMessage.readInt();
    }
    else {
      inputFailCode = INPUT_NUM_WORDS_FAIL;
      inputNumFails++;
      return;
    }
  }

  //Check checksum
  if (sumArray(inputNext, 1, INPUT_NUM_WORDS-1) != inputNext[INPUT_CHECKSUM]) {
    inputFailCode = INPUT_CHECKSUM_FAIL;
    inputNumFails++;
    return;
  }

  //Check sequence
  if (inputNext[INPUT_SEQUENCE] != inputArray[INPUT_SEQUENCE] + 1) {
    inputNumFails++;
  }
/*
  cmdLift = constrain(inputNext[INPUT_LIFT], 0, 999);
  cmdYaw = inputNext[INPUT_YAW]; 
  cmdPitch = inputNext[INPUT_PITCH];
  cmdRoll = inputNext[INPUT_ROLL];
  cmdButtons = inputNext[INPUT_BUTTON];
*/

  //Copy next to last
  for (int i = 0; i < INPUT_NUM_WORDS; i++) {
    inputArray[i] = inputNext[i];
  }
  newMessage = true;
}


//Get the status of a button number from the Command Message
int inputButtonStatus(int button) {
  return bitRead(inputArray[INPUT_BUTTON], button); 
}
