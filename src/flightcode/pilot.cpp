#include "pilot.h"

Pilot::Pilot() {
}

void Pilot::Fly(int *input, int *motorVal) {
    for (int i=0; i<3; i++) {
        motorVal[i] = input[i];   // Get input from Comm and update System.
    }
}


