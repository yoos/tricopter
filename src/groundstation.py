#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
from threading import Timer, Thread
from signal import signal, SIGINT

# ROS
import roslib; roslib.load_manifest("tricopter")
import rospy
from sensor_msgs.msg import Joy

rospy.init_node("tricopter_ground_station", anonymous=True)

# Tricopter
import gsconfig as cfg

print cfg.test

