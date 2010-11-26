#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import Qt
import serial

# ROS stuff
import roslib; roslib.load_manifest("tricopter")
import rospy
from joy.msg import Joy
from std_msgs.msg import String

rospy.init_node("tric_listener", anonymous=True)

# Comm values
serialPort = "/dev/ttyUSB0"
baudRate = 9600
freq = 10 # rospy.Rate(10) # 50 Hz
dogBone = chr(255) # Feed watchdog

time = rospy.get_time()

# Variables
xAxis = 0
yAxis = 1


# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.loginfo("Arduino at %s", serialPort)
except:
    rospy.loginfo("No Arduino!")

def feedDog():
    try:
        ser.write(dogBone)
        rospy.loginfo("Dog fed")
    except:
        rospy.loginfo("Unable to feed dog!")

def watchdogFeeder():
    global time
    if rospy.get_time() - time > 1/freq:
        feedDog()
        time = rospy.get_time()
        rospy.loginfo("ROSTime: %s", time)

############################ ROS get joystick input ###########################

def callback(myJoy):
    xAxisValue = int(255*(myJoy.axes[xAxis]+1)/2) # Range 0-255 in order to send as char value
    yAxisValue = int(255*(myJoy.axes[yAxis]+1)/2)
    rospy.loginfo("Axis 0: %s (%s)   Axis 1: %s (%s)", xAxisValue, chr(xAxisValue), yAxisValue, chr(yAxisValue))
    rospy.loginfo("Axis 0: %s   Axis 1: %s", myJoy.axes[xAxis], myJoy.axes[yAxis])
    try:
        ser.write(chr(yAxisValue))
    except:
        rospy.loginfo("ERROR: Unable to send data. Check connection.")

def listener():
    while not rospy.is_shutdown(): # TODO: make feedDog() loop at 50 Hz but not the callback.
        #rospy.Subscriber("joy", Joy, callback)

        watchdogFeeder()
        #freq.sleep()

#################################### Qt GUI ###################################

class MainWindow(Qt.QMainWindow):
    def __init__(self, *args):
        apply(Qt.QMainWindow.__init__, (self,) + args)

class CtrlButton(Qt.QPushButton):
    def __init__(self, *args):
        apply(Qt.QPushButton.__init__, (self,) + args)
        self.connect(self, SIGNAL("clicked()"), self.doPrint)
    def doPrint(self):
        rospy.loginfo("Control button clicked")

#mainWin = MainWindow()

#################################### Main #####################################

if __name__ == "__main__" :
    listener()
    #mainWin.show()

