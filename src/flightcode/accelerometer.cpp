#include "accelerometer.h"

Accelerometer::Accelerometer(int pinX, int pinY, int pinZ) {
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinZ, INPUT);
    pin[0] = pinX;
    pin[1] = pinY;
    pin[2] = pinZ;
}

float Accelerometer::get(int axis) {
    return analogRead(pin[axis]);
}

float Accelerometer::getX() {
    return Accelerometer::get(0);
}

float Accelerometer::getY() {
    return Accelerometer::get(1);
}

float Accelerometer::getZ() {
    return Accelerometer::get(2);
}

