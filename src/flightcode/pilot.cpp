#include "pilot.h"

Pilot::Pilot() {
    #ifdef DEBUG
    Serial.println("Pilot here!");
    #endif
}

void Pilot::Fly(int *input, int &mySystem) {
    for (int i=0; i<3; i++) {

        mySystem.SetMotor(i, input[i]);   // Get input from Comm and update System.
    }
    #ifdef DEBUG
    Serial.println("Pilot gave input to system.");
    #endif
}


