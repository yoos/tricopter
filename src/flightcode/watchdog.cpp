#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    deadCycle = 0;
    time = millis();
    #ifdef DEBUG
    Serial.println("Watchdog here!");
    #endif
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (isAlive && !seeFood && millis() - time > dogLife) {
        isAlive = false;
        Serial.println("Watchdog died!");
    }
//  else if (!isAlive && !seeFood) {   // This way, serial line is not spammed with long message.
//      Serial.print(".");
//      deadCycle++;
//      if (deadCycle > 80) {
//          Serial.println("");
//          deadCycle = 0;
//      }
//  }
    else if (seeFood) {
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;
        
        #ifdef DEBUG
        Serial.println("Watchdog received dogbone!");
        #endif
        }
}

