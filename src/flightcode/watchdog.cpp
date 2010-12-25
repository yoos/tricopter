#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    time = millis();
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (!seeFood && millis() - time > dogLife)
        isAlive = false;
    else if (seeFood) {
        #ifdef DEBUG
            Serial.print("Watchdog received dogbone!");
        #endif
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;
        }
}

