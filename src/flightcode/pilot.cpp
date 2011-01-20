#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    
    // Assume serial inputs
    serInput[SX] = 125;
    serInput[SY] = 125;
    serInput[ST] = 125;
    serInput[SZ] = 0;

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
                serInput[i] = 0;   // If something weird happens, assume input is zero.
            }
        }
    }
}

void Pilot::Fly(System &mySystem) {
    axisVal[SX] = serInput[SX] - 126;   // [-124, 124]
    axisVal[SY] = serInput[SY] - 126;   // [-124, 124]
    axisVal[ST] = serInput[ST] - 126;   // [-124, 124]
    axisVal[SZ] = serInput[SZ] - 2;         // [0, 248]

//  dir = atan2(axisVal[SY], axisVal[SX]);   // May need this eventually for IMU.

    // Intermediate calculations
    motorVal[MT] = axisVal[SZ] + 0.6667*axisVal[SY];   // Watch out for floats vs. ints
    motorVal[MR] = axisVal[SZ] - 0.3333*axisVal[SY] - axisVal[SX]/sqrt(3);
    motorVal[ML] = axisVal[SZ] - 0.3333*axisVal[SY] + axisVal[SX]/sqrt(3);

    // Find max/min motor values
    mapUpper = motorVal[MT] > motorVal[MR] ? motorVal[MT] : motorVal[MR];
    mapUpper = mapUpper > motorVal[ML] ? mapUpper : motorVal[ML];
    mapLower = motorVal[MT] < motorVal[MR] ? motorVal[MT] : motorVal[MR];
    mapLower = mapLower < motorVal[ML] ? mapLower : motorVal[ML];

    // Find map boundaries (need to limit, but NOT fit, to [0, 124])
    mapUpper = mapUpper > 124 ? mapUpper : 124;
    mapLower = mapLower < 0 ? mapLower : 0;

    // Final calculations
    for (int i=0; i<3; i++) {
        motorVal[i] = map(motorVal[i], mapLower, mapUpper, TMIN, TMAX);
    }
    tailServoVal = 90 + 0.1*axisVal[ST];

    // Write to motors
    for (int i=0; i<3; i++) {
        mySystem.SetMotor(i, motorVal[i]);
        Serial.print(motorVal[i]);
        Serial.print("   ");
    }
    mySystem.SetServo(tailServoVal);
    Serial.print(tailServoVal);
    Serial.println("");

    #ifdef DEBUG
    Serial.println("Pilot is flying.");
    #endif
}

