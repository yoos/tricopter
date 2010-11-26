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

serialPort = "/dev/ttyUSB0"
baudRate = 9600
freq = 50
dogBone = chr(255) # Feed watchdog

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.loginfo("Arduino at %s", serialPort)
except:
    rospy.loginfo("No Arduino!")

def feedDog():
    try:
        ser.write(dogBone)
    except:
        pass

############################ ROS get joystick input ###########################

def callback(myJoy):
    rospy.loginfo("Axis 0: %s   Axis 1: %s", myJoy.axes[0], myJoy.axes[1])
    try:
        ser.write(str(myJoy.axes[1]))
    except:
        rospy.loginfo("ERROR: Unable to send data. Check connection.")

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
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

