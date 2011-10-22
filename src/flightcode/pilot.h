#ifndef PILOT_H
#define PILOT_H

#include "globals.h"

class Pilot {
    char commStr[];
    int serRead;
    int serInput[PACKETSIZE];
    int axisVal[PACKETSIZE];
    int motorVal[3];
    int tailServoVal;
    int mapLower, mapUpper;
//  double dir;   // Direction in radians
    bool okayToFly;

public:
    Pilot();
    int Send(char[]);
    char* Read(char[]);
    void Listen();
    void Fly();
    void Abort();
    
    bool hasFood;
};

#endif // PILOT_H

