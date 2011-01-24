#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    long int time;   // Make sure this doesn't overflow!
    int deadCycle;
    int dogLife;

public:
    Watchdog(int);
    void Watch(bool&);
    bool isAlive;
};

#endif

