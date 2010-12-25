#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    time = millis();
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (!seeFood && millis() - time > dogLife)
        isAlive = false;
    else if (seeFood) {
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;
        }
}

