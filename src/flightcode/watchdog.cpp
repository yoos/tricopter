#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    time = millis();
}

void Watchdog::watch() { // If this runs too late, dogLife may already have ended.
    if (millis() - time < dogLife) {
        isAlive = true;
    else
        isAlive = false;
    }
}

void Watchdog::feed(char dogBone) {
    if (dogBone == DOGBONE) {
        time = millis();
    }
}

