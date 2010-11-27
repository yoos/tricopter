#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

class Accelerometer {
    int pin[]; // Pin numbers

public:
    Accelerometer(int, int, int);
    float get(int); // Reads from axis[]
    float getX(); // Alias to getAxis(0)
    float getY(); // Alias to getAxis(1)
    float getZ(); // Alias to getAxis(2)
};



#endif

