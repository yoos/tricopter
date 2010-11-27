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
baudRate = 57600
feedInterval = 0.1   # How often to feed watchdog in seconds
dogBone = chr(255) # Feed watchdog
logOn = False

# Variables
xAxis = 0
yAxis = 1


# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.loginfo("Arduino at %s", serialPort)
except:
    rospy.logerr("No Arduino!")

################################### Watchdog ##################################

def feedDog():
    try:
        ser.write(dogBone)
        rospy.loginfo("Dog fed")
    except:
        if logOn: rospy.logerr("Unable to feed dog!")

############################ ROS get joystick input ###########################

def callback(myJoy):
    xAxisValue = int(255*(myJoy.axes[xAxis]+1)/2)   # Range 0-255 in order to send as char value
    yAxisValue = int(255*(myJoy.axes[yAxis]+1)/2)
    if logOn: rospy.loginfo("Axis 0: %s (%s)   Axis 1: %s (%s)", xAxisValue, chr(xAxisValue), yAxisValue, chr(yAxisValue))
    if logOn: rospy.loginfo("Axis 0: %s   Axis 1: %s", myJoy.axes[xAxis], myJoy.axes[yAxis])
    try:
        ser.write(chr(yAxisValue))   # Testing. Need to try out ESC control.
    except:
        if logOn: rospy.logerr("ERROR: Unable to send data. Check connection.")

def listener():
    timeLast = rospy.Time.now()
    while not rospy.is_shutdown():
        if rospy.Time.now() - timeLast > rospy.Duration(feedInterval):   # Time - Time = Duration
            rospy.Subscriber("joy", Joy, callback)
            feedDog()
            timeLast = rospy.Time.now() # Update time
            if logOn: rospy.loginfo("ROSTime: %s", timeLast)
            #feedInterval.sleep()   # Maybe? Right now the manual time checks work.

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

