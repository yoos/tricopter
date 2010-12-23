#ifndef COMM_H
#define COMM_H

#include "globals.h"

class Communicator {
    char commStr[];
    int strLength;

public:
    Communicator();
    int Send(char[]);
    char* Read(char[]);
};

#endif

