#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    time = millis();
    #ifdef DEBUG
    Serial.println("Watchdog here!");
    #endif
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (!seeFood && millis() - time > dogLife) {
        isAlive = false;
        
        #ifdef DEBUG
        Serial.println("Watchdog dead!");
        #endif
    }
    else if (seeFood) {
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;
        
        #ifdef DEBUG
        Serial.println("Watchdog received dogbone!");
        #endif
        }
}

