#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    int time;
    int dogLife;
    bool isAlive;

public:
    Watchdog();
    void watch(int);
    void feed(char);

#endif

