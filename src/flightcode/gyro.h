#ifndef GYRO_H
#define GYRO_H

class Gyro {
    int pin[];

public:
    Gyro(int, int, int);
    float get(int);
    float getX();
    float getY();
    float getZ();

};

#endif

