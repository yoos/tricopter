#include "watchdog.h"

Watchdog::Watchdog(int timeout, char food) {
    dogLife = timeout;
    dogBone = food;
    time = millis();
}

void Watchdog::watch() { // If this runs too late, dogLife may already have ended.
    if (millis() - time < dogLife)
        isAlive = true;
    else
        isAlive = false;
}

void Watchdog::feed(char food) {
    if (food == dogBone)
        time = millis();
}

