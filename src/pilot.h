/*! \file pilot.h
 *  \author Soo-Hyun Yoo
 *  \brief Header for Pilot class.
 *
 *  Details.
 */

#ifndef PILOT_H
#define PILOT_H

#include "globals.h"
#include "joyinput.h"
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

public:
    Pilot();
    void Listen();
    void Talk();
    void Fly();
    void Abort();

    bool hasFood;

    void process_joystick_buttons() {
        // "Reset" targetRot[2] to currentRot[2] if thumb button is pressed.
        if (joy.buttons[BUTTON_RESET_YAW]) {
            targetRot[2] = currentRot[2];
            targetRot[2] += joy.axes[ST]/125 * Z_ROT_SPEED;
        }

        // Zero integral.
        if (joy.buttons[BUTTON_ZERO_INTEGRAL]) {
            PID[PID_ROT_X].integral = 0;
            PID[PID_ROT_Y].integral = 0;
        }

        // Trim throttle value.
        if (joy.buttons[BUTTON_DECREASE_TRIM] && joy.buttons[BUTTON_INCREASE_TRIM]) {
            throttleTrim = 0;
        }
        else if (joy.buttons[BUTTON_DECREASE_TRIM]) {
            throttleTrim--;
        }
        else if (joy.buttons[BUTTON_INCREASE_TRIM]) {
            throttleTrim++;
        }

        // Adjust gains on-the-fly.
        if (joy.buttons[BUTTON_DECREASE_XY_P_GAIN] && joy.buttons[BUTTON_INCREASE_XY_P_GAIN]) {
            PID[PID_ROT_X].P = XY_P_GAIN;
            PID[PID_ROT_Y].P = XY_P_GAIN;
        }
        else if (joy.buttons[BUTTON_DECREASE_XY_P_GAIN] && PID[PID_ROT_X].P > 0) {
            PID[PID_ROT_X].P -= 1.0;
            PID[PID_ROT_Y].P -= 1.0;
        }
        else if (joy.buttons[BUTTON_INCREASE_XY_P_GAIN]) {
            PID[PID_ROT_X].P += 1.0;
            PID[PID_ROT_Y].P += 1.0;
        }

        if (joy.buttons[BUTTON_DECREASE_XY_I_GAIN] && joy.buttons[BUTTON_INCREASE_XY_I_GAIN]) {
            PID[PID_ROT_X].I = XY_I_GAIN;
            PID[PID_ROT_Y].I = XY_I_GAIN;
        }
        else if (joy.buttons[BUTTON_DECREASE_XY_I_GAIN] && PID[PID_ROT_X].I > 0) {
            PID[PID_ROT_X].I -= 1.0;
            PID[PID_ROT_Y].I -= 1.0;
        }
        else if (joy.buttons[BUTTON_INCREASE_XY_I_GAIN]) {
            PID[PID_ROT_X].I += 1.0;
            PID[PID_ROT_Y].I += 1.0;
        }

        if (joy.buttons[BUTTON_DECREASE_XY_D_GAIN] && joy.buttons[BUTTON_INCREASE_XY_D_GAIN]) {
            PID[PID_ROT_X].D = XY_D_GAIN;
            PID[PID_ROT_Y].D = XY_D_GAIN;
        }
        else if (joy.buttons[BUTTON_DECREASE_XY_D_GAIN] && PID[PID_ROT_X].D < 0) {
            PID[PID_ROT_X].D += 1.0;
            PID[PID_ROT_Y].D += 1.0;
        }
        else if (joy.buttons[BUTTON_INCREASE_XY_D_GAIN]) {
            PID[PID_ROT_X].D -= 1.0;
            PID[PID_ROT_Y].D -= 1.0;
        }
    }

};

#endif // PILOT_H

