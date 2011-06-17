#!/usr/bin/env python

import serial

ser = serial.Serial("/dev/ttyUSB0", 19200)

while 1:
    print(ser.readline())

