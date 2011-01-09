#ifndef PILOT_H
#define PILOT_H

#include "globals.h"
#include "system.h"

class Pilot {
    char commStr[];
    int serRead;
    int serInput[PACKETSIZE];
    int motorVal[3];
    double dir;   // Direction in radians

public:
    Pilot();
    int Send(char[]);
    char* Read(char[]);
    void Listen();
    void Fly(System&);
    
    bool hasFood;
};

#endif // PILOT_H

