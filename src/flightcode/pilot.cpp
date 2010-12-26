#include "pilot.h"

Pilot::Pilot() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    for (int i=0; i<PACKETSIZE; i++)
        input[i] = 0;
    #ifdef DEBUG
    Serial.println("Comm here!");
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
            Serial.print("Comm received dogbone: ");
            Serial.println(serRead);
            #endif
            hasFood = true;   // Will be set to false by watchdog.
        }
        else if (serRead == SERHEAD) {   // Receive header.
            #ifdef DEBUG
            Serial.print("Comm received header: ");
            Serial.println(serRead);
            #endif
            for (int i=0; i<2; i++) {   // Each packet consists of header plus two bytes.
                serRead = Serial.read();

                #ifdef DEBUG
                Serial.print("Comm received motor control byte #");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(serRead);
                #endif

                if (serRead == SERHEAD | serRead == NULL) {   // Dropped byte?
                    i = 0;   // Discard and start over.
                    Serial.println("Comm detected packet drop.");
                }
                else if (serRead == DOGBONE)   // The dogbone might be sent in the middle of a control packet.
                    hasFood = true;
                else {
                    input[i] = map(serRead, 0, 250, THROTTLE_MIN, THROTTLE_MAX);   // If all is good, write to inputlist.
                    #ifdef DEBUG
                    Serial.println("Comm determined motor value.");
                    #endif
                }
            }
        }
        else {
            for (int i=0; i<PACKETSIZE; i++) {
                input[i] = 16;   // If something weird happens, throttle down.
            }
        }
    }
}

void Pilot::Fly(System &mySystem) {
    for (int i=0; i<3; i++) {
        mySystem.SetMotor(i, input[i]);
    }
    #ifdef DEBUG
    Serial.println("Pilot gave input to system.");
    #endif
}

