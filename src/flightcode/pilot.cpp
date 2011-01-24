#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    
    // Assume serial inputs
    serInput[SX] = 126;
    serInput[SY] = 126;
    serInput[ST] = 126;
    serInput[SZ] = 126;   // Keep this at non-zero to prevent disastrous false starts.

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
        delayMicroseconds(200);

        if (serRead == SERHEAD) {   // Receive header.
            hasFood = true;
            #ifdef DEBUG
            Serial.print("Pilot received header: ");
            Serial.println(serRead);
            #endif
            for (int i=0; i<PACKETSIZE; i++) {
                serRead = Serial.read();
                delayMicroseconds(200);

                #ifdef DEBUG
                Serial.print("Pilot received motor control byte #");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(serRead);
                #endif

                if (serRead == SERHEAD) {   // Dropped byte?
//                  i = 0;   // Discard and start over.
                    Serial.println("Pilot detected packet drop.");
                }
                else if (serRead == -1) {   // This happens when serial is empty.
                    Serial.println("Pilot detected malformed packet.");
                    digitalWrite(13, HIGH);
                }
                else if (serRead >= INPUT_MIN && serRead <= INPUT_MAX) {
                    serInput[i] = serRead;
                    #ifdef DEBUG
                    Serial.println("Pilot determined motor value.");
                    #endif
                    digitalWrite(13, LOW);
                }
            }
        }
        else {
            Serial.print("Weird header!");   // Warn if something weird happens.
            Serial.println("");
        }
    }
}

void Pilot::Fly(System &mySystem) {
    axisVal[SX] = serInput[SX] - 126;   // [-125, 125]
    axisVal[SY] = serInput[SY] - 126;   // [-125, 125]
    axisVal[ST] = serInput[ST] - 126;   // [-125, 125]
    axisVal[SZ] = serInput[SZ] - 1;     // [0, 250]

    for (int i=0; i<4; i++) {
        Serial.print(serInput[i]);
        Serial.print("   ");
    }
    Serial.println("");

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

    // Find map boundaries (need to limit, but NOT fit, to [1, 125])
    mapUpper = mapUpper > 250 ? mapUpper : 250;
    mapLower = mapLower < 0 ? mapLower : 0;

    // Final calculations
    for (int i=0; i<3; i++) {
        motorVal[i] = map(motorVal[i], mapLower, mapUpper, TMIN, TMAX);
    }
    tailServoVal = 100 - 0.2*axisVal[ST];

    // Write to motors
    for (int i=0; i<3; i++) {
        mySystem.SetMotor(i, motorVal[i]);
//      Serial.print(motorVal[i]);
//      Serial.print("   ");
    }
//  mySystem.SetServo(tailServoVal);
//  Serial.print(tailServoVal);
//  Serial.println("");

    #ifdef DEBUG
    Serial.println("Pilot is flying.");
    #endif
}

