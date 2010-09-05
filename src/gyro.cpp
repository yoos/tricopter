#include "gyro.h"

Gyro::Gyro(int pinX, int pinY, int pinZ) {
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinZ, INPUT);
    pin[0] = pinX;
    pin[1] = pinY;
    pin[2] = pinZ;
}

float Gyro::get(int axis) {
    return analogRead(pin[axis]);
}

float Gyro::getX() {
    return Gyro::get(0);
}

float Gyro::getY() {
    return Gyro::get(1);
}

float Gyro::getZ() {
    return Gyro::get(2);
}

