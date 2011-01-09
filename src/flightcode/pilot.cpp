#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    for (int i=0; i<PACKETSIZE; i++) {
        serInput[i] = 0;
    }
    #ifdef DEBUG
    Serial.println("Pilot here!");
    #endif
}

int Pilot::Send(char sStr[]) {
    zeroStr(commStr);
    for (int i=0; i<128; i++) {
        commStr[i] = sStr[i];
    }
   
    Serial.println(commStr);

    return 0;
}

char* Pilot::Read(char sStr[]) {
    zeroStr(commStr);
    int i = 0;
    while (Serial.available() > 0) {
        commStr[i] = sStr[i];
        i++;
    }

    return commStr;
}

void Pilot::Listen() {
    while (Serial.available()) {
        serRead = Serial.read();

        if (serRead == DOGBONE) {   // Receive dogbone.
            #ifdef DEBUG
            Serial.print("Pilot received dogbone: ");
            Serial.println(serRead);
            #endif
            hasFood = true;   // Will be set to false by watchdog.
        }
        else if (serRead == SERHEAD) {   // Receive header.
            #ifdef DEBUG
            Serial.print("Pilot received header: ");
            Serial.println(serRead);
            #endif
            for (int i=0; i<PACKETSIZE; i++) {
                serRead = Serial.read();

                #ifdef DEBUG
                Serial.print("Pilot received motor control byte #");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(serRead);
                #endif

                if (serRead == SERHEAD | serRead == NULL) {   // Dropped byte?
                    i = 0;   // Discard and start over.
                    Serial.println("Pilot detected packet drop.");
                }
                else if (serRead == DOGBONE) {   // The dogbone might be sent in the middle of a control packet.
                    hasFood = true;
                    i--;
                }
                else {
                    serInput[i] = serRead;
                    #ifdef DEBUG
                    Serial.println("Pilot determined motor value.");
                    #endif
                }
            }
        }
        else {
            for (int i=0; i<PACKETSIZE; i++) {
                motorVal[i] = THROTTLE_MIN;   // If something weird happens, throttle down.
            }
        }
    }
}

void Pilot::Fly(System &mySystem) {
    axisVal[SX] = serInput[SX] - 125;
    axisVal[SY] = serInput[SY] - 125;
    axisVal[ST] = serInput[ST] - 125;   // Comment out when testing with two-axis joystick!
    axisVal[SZ] = serInput[SZ];

//  dir = atan2(axisVal[SY], axisVal[SX]);   // May need this eventually for IMU.

    // Intermediate calculations
    motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs. ints
    motorVal[MR] = axisVal[SZ] - 0.3333*axisVal[SY] - axisVal[SX]/sqrt(3);
    motorVal[ML] = axisVal[SZ] - 0.3333*axisVal[SY] + axisVal[SX]/sqrt(3);

    // Normalization
    motorMax = motorVal[MT] > motorVal[MR] ? motorVal[MT] : motorVal[MR];
    motorMax = motorMax > motorVal[ML] ? motorMax : motorVal[ML];
    if (motorMax > 125) {
        for (int i=0; i<3; i++) {
            motorVal[i] = map(motorVal[i], 1, motorMax, 0, 125);   // Cap at 125.
        }
    }

    // Final calculations
    for (int i=0; i<3; i++) {
        motorVal[i] = map(motorVal[i], 0, 125, THROTTLE_MIN, THROTTLE_MAX);
    }

    // Write to motors
    for (int i=0; i<3; i++) {
        mySystem.SetMotor(i, motorVal[i]);
        Serial.print(motorVal[i]);
        Serial.print("   ");
    }
    Serial.println("");

    #ifdef DEBUG
    Serial.println("Pilot is flying.");
    #endif
}

