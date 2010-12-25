#ifndef SYSTEM_H
#define SYSTEM_H

#include <Servo.h>
#include "comm.h"

class System {
    Servo motor[3];

public:
    System();
    void Run();
    void Die();
    int motorVal[3];
};



#endif
