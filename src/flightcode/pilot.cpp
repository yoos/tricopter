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
    axisVal[ST] = 0; //serInput[ST] - 125;
    axisVal[SZ] = 0; //serInput[SZ];
    Serial.print(axisVal[SX]);
    Serial.print("   ");
    Serial.print(axisVal[SY]);
    Serial.print("   ");

    dir = atan2(axisVal[SY], axisVal[SX]);
    Serial.print(dir);
    Serial.print("   ");
//  Serial.println(axisVal[SX]);

    motorVal[MT] = axisVal[SZ] + axisVal[SY]*cos(dir)        + axisVal[SX]*sin(dir);
    motorVal[ML] = axisVal[SZ] + axisVal[SY]*cos(dir+2/3*PI) + axisVal[SX]*sin(dir+2/3*PI);
    motorVal[MR] = axisVal[SZ] + axisVal[SY]*cos(dir-2/3*PI) + axisVal[SX]*sin(dir-2/3*PI);

//  for (int i=0; i<3; i++) {
//      motorVal[i] = map(motorVal[i], INPUT_MIN, INPUT_MAX, THROTTLE_MIN, THROTTLE_MAX);   // If all is good, write to inputlist.
//  }

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

