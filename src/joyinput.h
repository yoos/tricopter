/*! \file joyinput.h
 *  \author Soo-Hyun Yoo
 *  \brief Process joystick input.
 *
 *  Details.
 */

#ifndef JOYINPUT_H
#define JOYINPUT_H

#include <globals.h>

struct joyInput {
    float axes[6];
    bool buttons[14];
} joy;

void update_joystick_input(uint8_t* serialInput) {
    // ========================================================================
    // Shift serial input values [0, 250] to correct range for each axis. Z
    // stays positive for ease of calculation.
    // ========================================================================
    joy.axes[SX] = (float) serialInput[SX] - 125;   // [-125, 125]
    joy.axes[SY] = (float) serialInput[SY] - 125;   // [-125, 125]
    joy.axes[ST] = (float) serialInput[ST] - 125;   // [-125, 125]
    joy.axes[SZ] = (float) serialInput[SZ];         // [   0, 250]

    // ========================================================================
    // Set button values. We utilize only the lower 7 bits, since doing
    // otherwise would cause overlaps with serial headers.
    // ========================================================================
    for (int i=0; i<7; i++) {
        joy.buttons[i]   = serialInput[SB1] & (1<<i);
        joy.buttons[7+i] = serialInput[SB2] & (1<<i);
    }
}

#endif // JOYINPUT_H

