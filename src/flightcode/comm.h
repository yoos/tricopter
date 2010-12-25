#ifndef COMM_H
#define COMM_H

#include "globals.h"

class Communicator {
    char commStr[];
    int serRead;

public:
    Communicator();
    int Send(char[]);
    char* Read(char[]);
    void Listen();
    
    bool hasFood;
    
    int input[PACKETSIZE];
};

#endif

