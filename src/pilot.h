#ifndef PILOT_H
#define PILOT_H

#include "globals.h"
#include "pid.h"

class Pilot {
    int serRead;
    int serInput[PACKETSIZE];
    float axisVal[PACKETSIZE];
    float mapLower, mapUpper;
//  double dir;   // Direction in radians
    bool okayToFly;
    long numGoodComm;
    long numBadComm;

public:
    Pilot();
    void Listen();
    void Talk();
    void Fly();
    void Abort();
    
    bool hasFood;
};

#endif // PILOT_H

