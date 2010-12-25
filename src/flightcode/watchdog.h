#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    int time;
    int dogLife;

public:
    Watchdog(int);
    void Watch(bool&);
    bool isAlive;
};

#endif

