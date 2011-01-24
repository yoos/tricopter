#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import Qt
import serial

# ROS stuff
import roslib; roslib.load_manifest("tricopter")
import rospy
from joy.msg import Joy
from tricopter.msg import TricJoy

# Comm values
serialPort = "/dev/ttyUSB0"
baudRate = 57600
dataSendInterval = 0.05   # Interval between data sends in seconds
serHeader = chr(255)
okayToSend = False
verboseOn = True

armed = False

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
axisValues = [126, 126, 126, 126]   # Keep Z value at non-zero so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
buttonValues = []

rospy.init_node("tric_comm_node", anonymous=True)

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.logerr("Arduino at %s", serialPort)
except:
    rospy.logerr("No Arduino at %s!", serialPort)


############################ ROS get joystick input ###########################

def callback(myJoy):
    global axisValues, armed
    # Calculate axis values.
    axisValues[0] = int(250*(xSign * myJoy.axes[xAxis] + 1) / 2 + 1)   # Range 1-251 in order to send as char value
    axisValues[1] = int(250*(ySign * myJoy.axes[yAxis] + 1) / 2 + 1)
    axisValues[2] = int(250*(tSign * myJoy.axes[tAxis] + 1) / 2 + 1)
    axisValues[3] = int(250*(zSign * myJoy.axes[zAxis] + 1) / 2 + 1)

    if armed:
        sendData()
    elif not armed:
        rospy.logerr("Move joystick throttle to minimum position in order to send motor arming signal.")
        if axisValues[3] == 1:   # If throttle is at minimum position
            armed = True
            rospy.logerr("Joystick throttle at minimum! Motors armed!")
    
    rospy.sleep(dataSendInterval)

def sendData():
    try:
        ser.write(serHeader + chr(axisValues[0]) + chr(axisValues[1]) + chr(axisValues[2]) + chr(axisValues[3]))
        if verboseOn: rospy.logerr("A0: %s   A1: %s   A2: %s   A3: %s", axisValues[0], axisValues[1], axisValues[2], axisValues[3])
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")

def tric_comm():
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback, queue_size=1)
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
        rospy.logerr("Control button clicked")

#mainWin = MainWindow()

#################################### Main #####################################

if __name__ == "__main__" :
    try:
        tric_comm()
    except rospy.ROSInterruptException:
        pass
    #mainWin.show()

