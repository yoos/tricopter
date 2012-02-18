/*! \file watchdog.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for Watchdog class.
 *
 *  Details.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    long int time;   // Make sure this doesn't overflow!
    int deadCycle;
    int dogLife;

public:
    Watchdog(int);
    void watch(bool&);
    bool isAlive;
};

#endif

