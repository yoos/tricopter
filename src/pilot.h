/*! \file pilot.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for Pilot class.
 *
 *  Details.
 */

#ifndef PILOT_H
#define PILOT_H

#include "globals.h"
#include "pid.h"

class Pilot {
    byte serRead;
    byte serInput[PACKETSIZE];
    float axisVal[4];
    bool buttonVal[14];
    int throttleTrim;
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

