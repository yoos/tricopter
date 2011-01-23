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
baudRate = 19200
inputRate = 20   # Input rate in Hz
serHeader = chr(255)
dogBone = chr(254) # Feed watchdog
okayToSend = False
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
xValue = 126
yValue = 126
tValue = 126
zValue = 126   # Keep this at non-zero so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.

xZero = True
yZero = True
tZero = True
zZero = False

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
            if verboseOn: rospy.logerr("Dog fed")
            feedLast = rospy.Time.now() # Update time
        except:
            if verboseOn: rospy.logerr("Unable to feed dog!")

############################ ROS get joystick input ###########################

def callback(myJoy):
    global okayToSend, xValue, yValue, zValue, tValue

    # Calculate axis values.
    xValue = int(250*(xSign * myJoy.axes[xAxis] + 1) / 2 + 1)   # Range 1-251 in order to send as char value
    yValue = int(250*(ySign * myJoy.axes[yAxis] + 1) / 2 + 1)
    tValue = int(250*(tSign * myJoy.axes[tAxis] + 1) / 2 + 1)
    zValue = int(250*(zSign * myJoy.axes[zAxis] + 1) / 2 + 1)

    okayToSend = True
        
def sendData():
    try:
        ser.write(serHeader + chr(xValue) + chr(yValue) + chr(tValue) + chr(zValue))
        if verboseOn: rospy.logerr("A0: %s   A1: %s   A2: %s   A3: %s", xValue, yValue, tValue, zValue)
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")


def listener():
    global okayToSend
    rate = rospy.Rate(inputRate)
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        if okayToSend:
            sendData()
        rate.sleep()
        #rospy.spin()

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

