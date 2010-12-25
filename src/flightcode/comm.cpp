#include "comm.h"

Communicator::Communicator() {
    Serial.begin(BAUDRATE);
    hasFood = false;
    for (int i=0; i<PACKETSIZE; i++)
        input[i] = 0;
}

int Communicator::Send(char sStr[]) {
    zeroStr(commStr);
    for (int i=0; i<128; i++) {
        commStr[i] = sStr[i];
    }
   
    Serial.println(commStr);

    return 0;
}

char* Communicator::Read(char sStr[]) {
    zeroStr(commStr);
    int i = 0;
    while (Serial.available() > 0) {
        commStr[i] = sStr[i];
        i++;
    }

    return commStr;
}

void Communicator::Listen() {
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
                    Serial.print("Motor control byte #");
                    Serial.print(i);
                    Serial.print(": ");
                    Serial.println(serRead);
                #endif

                if (serRead == SERHEAD | serRead == NULL) {   // Dropped byte?
                    i = 0;   // Discard and start over.
                    Serial.println("Packet drop detected.");
                }
                else if (serRead == DOGBONE)   // The dogbone might be sent in the middle of a control packet.
                    hasFood = true;
                else {
                    input[i] = map(serRead, 0, 250, 0, 178);   // If all is good, write to inputlist.
                    #ifdef DEBUG
                        Serial.print("Motor ");
                        Serial.print(i);
                        Serial.print(" set to ");
                        Serial.print(input[i]);
                        Serial.print("   ");
                        if (i == 1) Serial.println("");   // Newline.
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



