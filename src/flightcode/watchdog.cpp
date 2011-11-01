#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    deadCycle = 0;
    time = millis();
    isAlive = false;
    #ifdef DEBUG
    serPrint("Watchdog here!\n");
    #endif
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (isAlive && !seeFood && millis() - time > dogLife) {
        isAlive = false;
        serPrint("Watchdog died!\n");
    }
//  else if (!isAlive && !seeFood) {   // This way, serial line is not spammed with long message.
//      serPrint(".");
//      deadCycle++;
//      if (deadCycle > 80) {
//          serPrint("\n");
//          deadCycle = 0;
//      }
//  }
    else if (seeFood) {
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;
        
        #ifdef DEBUG
        serPrint("Watchdog received dogbone!\n");
        #endif
        }
}

