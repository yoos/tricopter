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


# Comm values
serialPort = "/dev/ttyUSB0"
baudRate = 9600
feedInterval = 0.1   # How often to feed watchdog in seconds
inputInterval = 0.05   # How often to send input values
serHeader = chr(255)
dogBone = chr(254) # Feed watchdog
verboseOn = True

# Variables
xAxis = 0   # X axis
yAxis = 1   # Y axis
tAxis = 2   # Twist axis
zAxis = 3   # Z axis
hAxis = 4   # Top horizontal
vAxis = 5   # Top vertical

# Axis sign flips
xSign = -1
ySign = 1
tSign = -1
zSign = 1
hSign = -1
vSign = 1

# Axis values
xValue
yValue
tValue
zValue

rospy.init_node("tric_listener", anonymous=True)
feedLast = rospy.Time.now()
inputLast = rospy.Time.now()

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.loginfo("Arduino at %s", serialPort)
except:
    rospy.logerr("No Arduino at %s!", serialPort)

################################### Watchdog ##################################

def feedDog():
    global feedLast
    if rospy.Time.now() - feedLast > rospy.Duration(feedInterval):
        # Take care of the dog.
        if verboseOn: rospy.loginfo("ROSTime: %s", feedLast)
        try:
            ser.write(dogBone)
            if verboseOn: rospy.loginfo("Dog fed")
            feedLast = rospy.Time.now() # Update time
        except:
            if verboseOn: rospy.logerr("Unable to feed dog!")

############################ ROS get joystick input ###########################

def callback(myJoy):
    global inputLast, xValue, yValue, zValue, tValue
    if rospy.Time.now() - inputLast > rospy.Duration(inputInterval):   # Time - Time = Duration
        inputLast = rospy.Time.now()
        # Calculate axis values.
        xValue = int(250*(xSign * myJoy.axes[xAxis] + 1) / 2)   # Range 0-250 in order to send as char value
        yValue = int(250*(ySign * myJoy.axes[yAxis] + 1) / 2)
        tValue = int(250*(tSign * myJoy.axes[tAxis] + 1) / 2)
        zValue = int(250*(zSign * myJoy.axes[zAxis] + 1) / 2)

        # Write to serial.
        try:
            ser.write(serHeader + chr(xValue) + chr(yValue) + chr(tValue) + chr(zValue))
            if verboseOn: rospy.loginfo("A0: %s   A1: %s   A2: %s   A3: %s", xValue, yValue, zValue, tValue)
        except:
            if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")


def listener():
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        feedDog()
        rospy.spin()

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

