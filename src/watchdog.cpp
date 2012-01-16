/*! \file watchdog.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Simple watchdog class.
 *
 *  Details.
 */

#include "watchdog.h"

Watchdog::Watchdog(int timeout) {
    dogLife = timeout;
    deadCycle = 0;
    time = millis();
    isAlive = false;
    #ifdef DEBUG
    spln("Watchdog here!");
    #endif
}

void Watchdog::Watch(bool &seeFood) {   // If this runs too late, dogLife may already have ended.
    if (isAlive && !seeFood && millis() - time > dogLife) {
        isAlive = false;
        spln("Watchdog died!");
    }
//  else if (!isAlive && !seeFood) {   // This way, serial line is not spammed with long message.
//      sp(".");
//      deadCycle++;
//      if (deadCycle > 80) {
//          spln("");
//          deadCycle = 0;
//      }
//  }
    else if (seeFood) {
        seeFood = false;   // Eat the food.
        time = millis();
        isAlive = true;

        #ifdef DEBUG
        spln("Watchdog received dogbone!");
        #endif
        }
}

