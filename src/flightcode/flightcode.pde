#include "globals.h"
#include "watchdog.h"
#include "watchdog.cpp"

void setup() {
}

void loop() {
    Watchdog Jasper(3000, DOGBONE);
    while (Jasper.isAlive) {
        Jasper.watch();
        for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
            analogWrite(MOTOR_TEST_PIN, fadeValue);         
            delay(10);                            
        } 
        for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) { 
            analogWrite(MOTOR_TEST_PIN, fadeValue);         
            delay(10);                            
        } 
    }
    while (!Jasper.isAlive) {
        analogWrite(MOTOR_TEST_PIN, 255);
    }
}


