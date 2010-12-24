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

    uint8_t motorInput[2];
    for (;;) {
//      while (Jasper.isAlive) {
        Jasper.isAlive = true;
        int n = 0;
        while (true) {
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
                        else if (myChr == DOGBONE)   // The dogbone might be sent in the middle of a control packet.
                            Jasper.Feed();   // Feed dog and ignore discrepancy.
                        else
                            motorInput[i] = myChr;   // If all is good, write to motorInput.
                    }
                    Serial.print("X: ");
                    Serial.print((int) motorInput[0]);
                    Serial.print("  Y: ");
                    Serial.print((int) motorInput[1]);
                    Serial.print("   Writing ");
                    Serial.println((int) motorInput[0] * 180/250);
                    myServo.write(0);
                }
                else {
                    if (n < 178) {
                        myServo.write(n);
                        Serial.print("n = ");
                        Serial.println(n);
                        n++;
                    }
                }
 
//                  myServo.write(0);   // If there's some weird packet, send minimum throttle.
            }
            delay(50);
        }
        while (!Jasper.isAlive) {
        }
    }

    return 0;
}


