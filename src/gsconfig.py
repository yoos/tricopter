#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Communication
#serialPort = "/dev/ttyUSB0"   # Uncomment to specify serial port. Otherwise, will connect to first available port.
baudRate = 57600
dataSendInterval = 0.030   # 30 ms interval = 33.3 hz. NOTE: This frequency should be LOWER than the microcontroller's control loop frequency!
dogFeedInterval = 0.1
serHeader = chr(255)
dogBone = chr(254)

# Joystick axis sign flips.
axisSigns = [-1, 1, 1, 1, -1, 1]

# Joystick axis indices.
axisX = 0
axisY = 1
axisT = 2
axisZ = 3

