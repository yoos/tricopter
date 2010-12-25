#include <Servo.h>
#include <Wire.h>
#include "globals.h"
#include "comm.cpp"
#include "watchdog.cpp"
#include "itg3200.cpp"
#include "bma180.cpp"

int main(void) {
    init();   // For Arduino.
 
    // Begin Arduino services.
    Wire.begin();
    
    // Begin system services.
    Communicator Alice;
    Alice.Send("Antares starting!");
    Watchdog Jasper(3000, DOGBONE);   // Timeout in ms.
    BMA180 myAcc(4, 2);   // range, bandwidth: DS p. 27
    ITG3200 myGyr(2);   // 0, 1, 2, 3 are Reserved, Reserved, Reserved, 2000 deg/s.
    Servo myServo;

    // Attach motors.
    myServo.attach(9);

    for (;;) {
//      while (Jasper.isAlive) {
        Jasper.isAlive = true;
        unsigned int n = 0;
        unsigned int motorInput[2];
        while (true) {
            for (int i=0; i<2; i++) {
                if (motorInput[i] > 178) {
                    Serial.println("Input scaled!");
                    motorInput[i] = map(motorInput[i], 0, motorInput[i], 0, 178);
                }
            }
            Jasper.Watch();
//          myGyr.Poll();
//          myAcc.Poll();


            char myChr;
            if (Serial.available()) {
               myChr = Serial.read();
                
                if (myChr == DOGBONE) {
                    Jasper.Feed();
                    Alice.Send("Dogbone received!");
                }
                else if (myChr == SERHEAD) {
                    Alice.Send("Header received!");
                    for (int i=0; i<2; i++) {   // For now, only two bytes.
                        myChr = Serial.read();
                        if (myChr == SERHEAD) {   // If there was a dropped byte
                            i = 0;   // Start over
                            Alice.Send("Packet drop detected.");
                        }
                        else if (myChr == DOGBONE) {   // The dogbone might be sent in the middle of a control packet.
                            Jasper.Feed();   // Feed dog and ignore discrepancy.
                            Serial.println("Dogbone received!");
                        }
                        else {
                            motorInput[i] = map((uint8_t) myChr, 0, 250, 0, 178);   // If all is good, write to motorInput.
                            Serial.print("Motor ");
                            Serial.print(i);
                            Serial.print(" set to ");
                            Serial.print((uint8_t) motorInput[i]);
                            Serial.print("   ");
                            if (i)
                                Serial.println("");   // Newline.
                        }
                    }
                }
                else {
                    for (int i=0; i<2; i++) {
                        motorInput[i] = 160;   // If there's some weird packet, send minimum throttle.
                    }
                }
            }
            Serial.print("   Writing ");
            Serial.println((int) motorInput[1]);
//          myServo.writeMicroseconds(motorInput[0] * 180/250 * 1000);
            motorInput[1] = 150;
            myServo.write(motorInput[1]);
            delay(100);
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


