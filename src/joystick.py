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
feedInterval = 0.05   # How often to feed watchdog in seconds
inputInterval = 0.1   # How often to send input values
serHeader = chr(255)
dogBone = chr(254) # Feed watchdog
logOn = True

# Variables
xAxis = 0
yAxis = 1
xAxisValue = 125
yAxisValue = 125

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
        if logOn: rospy.loginfo("ROSTime: %s", feedLast)
        try:
            ser.write(dogBone)
            if logOn: rospy.loginfo("Dog fed")
            feedLast = rospy.Time.now() # Update time
        except:
            if logOn: rospy.logerr("Unable to feed dog!")

############################ ROS get joystick input ###########################

def callback(myJoy):
    global inputLast, xAxisValue, yAxisValue
    if rospy.Time.now() - inputLast > rospy.Duration(inputInterval):   # Time - Time = Duration
        # Calculate axis values.
        xAxisValue = int(250*(myJoy.axes[xAxis]+1)/2)   # Range 0-250 in order to send as char value
        yAxisValue = int(250*(myJoy.axes[yAxis]+1)/2)


def sendData():
    global xAxisValue, yAxisValue
    if logOn: rospy.loginfo("Axis 0: %s (%s)   Axis 1: %s (%s)", xAxisValue, chr(xAxisValue), yAxisValue, chr(yAxisValue))
    # Write to serial.
    try:
        ser.write(serHeader + chr(xAxisValue) + chr(yAxisValue))   # Testing. Need to try out ESC control.
    except:
        if logOn: rospy.logerr("ERROR: Unable to send data. Check connection.")


def listener():
#   while not rospy.is_shutdown():
#       if rospy.Time.now() - feedLast > rospy.Duration(feedInterval):
    r = rospy.Rate(1)   # 10 Hz
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        sendData()
        feedDog()
        r.sleep()
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

