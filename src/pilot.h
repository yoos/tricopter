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
    uint8_t serInput[PACKETSIZE];
    int throttleTrim;
    float mapLower, mapUpper;
//  double dir;   // Direction in radians
    bool okayToFly;
    long numGoodComm;
    long numBadComm;

    struct joyInput {
        float axes[6];
        bool buttons[14];
    } joy;

    void update_joystick_input(void);
    void process_joystick_buttons(void);

public:
    Pilot();
    void Listen();
    void Talk();
    void Fly();
    void Abort();

    bool hasFood;

};

#endif // PILOT_H

