#!/usr/bin/env python
# -*- coding: utf-8 -*-

# =============================================================================
# Communication
# =============================================================================

# General
#serialPort = "/dev/ttyUSB0"   # Uncomment to specify serial port. Otherwise, will connect to first available port.
baudRate = 57600
debug = False

# TX
dataSendInterval = 0.025   # 25 ms interval = 40 Hz. NOTE: This frequency should be LOWER than the microcontroller's control loop frequency!
dogFeedInterval = 0.1
serHeader = chr(255)
dogBone = chr(254)

# RX
newlineSerTag  = '\xde\xad\xbe\xef'
fieldSerTag    = '\xff\xff'
dcmSerTag      = '\xfb'
rotationSerTag = '\xfc'
motorSerTag    = '\xfd'
pidSerTag      = '\xfe'


# Joystick axis sign flips.
axisSigns = [-1, 1, 1, 1, -1, 1]

# Joystick axis indices.
axisX = 0
axisY = 1
axisT = 2
axisZ = 3

