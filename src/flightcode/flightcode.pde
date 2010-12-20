#include <Servo.h>
#include "globals.h"
#include "watchdog.cpp"

void setup() {
    Serial.begin(57600);
    Serial.println("Antares starting!");
}

void loop() {
    Watchdog Jasper(3000, DOGBONE);
    while (Jasper.isAlive) {
        Jasper.watch();
        for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
            analogWrite(MOTOR_TEST_PIN, fadeValue);         
            analogWrite(MTP2, fadeValue);
            delay(10);                            
        } 
        for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) { 
            analogWrite(MOTOR_TEST_PIN, fadeValue);         
            analogWrite(MTP2, fadeValue);
            delay(10);                            
        } 
    }
    while (!Jasper.isAlive) {
        analogWrite(MOTOR_TEST_PIN, 255);
    }
}


