#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    int time;
    int dogLife;
    bool isAlive;

public:
    Watchdog(int);
    void watch();
    void feed(char);
};

#endif

