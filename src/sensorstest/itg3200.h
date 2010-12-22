#ifndef GYRO_H
#define GYRO_H

#include "i2c.h"
#include "globals.h"

#define GYRADDR 0x69 // gyro address, binary = 11101001
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

#define READ_SIZE 6

#define REGADDR 0x1D

class ITG3200 {
    uint16_t gBuffer[READ_SIZE];
    char gStr[512];

    uint16_t gRaw[3];
    float gVal[3];

public:
    ITG3200();
    void Poll();
    float* Get();
    float Get(int);
};

