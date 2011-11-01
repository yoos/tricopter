#!/usr/bin/env python

import serial

ser = serial.Serial("/dev/ttyUSB0", 57600)

while 1:
    while ser.inWaiting() > 0:
        print(ser.readline())

