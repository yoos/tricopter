#ifndef PILOT_H
#define PILOT_H

#include "globals.h"
#include "system.h"

class Pilot {
    char commStr[];
    int serRead;
    int serInput[PACKETSIZE];
    int axisVal[PACKETSIZE];
    int motorVal[3];
    int tailServoVal;
    int mapLower, mapUpper;
//  double dir;   // Direction in radians

public:
    Pilot();
    int Send(char[]);
    char* Read(char[]);
    void Listen();
    void Fly(System&);
    void Abort(System&);
    
    bool hasFood;
};

#endif // PILOT_H

