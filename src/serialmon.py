#!/usr/bin/env python

import serial

ser = serial.Serial("/dev/ttyUSB0", 19200)

while 1:
    while ser.inWaiting() > 0:
        print(ser.readline())

