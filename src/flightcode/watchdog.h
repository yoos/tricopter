#ifndef WATCHDOG_H
#define WATCHDOG_H

class Watchdog {
    int time;
    int dogLife;
    char dogBone;

public:
    Watchdog(int, char);
    void watch();
    void feed(char);
    bool isAlive;
};

#endif

