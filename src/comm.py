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

rospy.init_node("tric_comm_node", anonymous=True)

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.logerr("Arduino at %s", serialPort)
except:
    rospy.logerr("No Arduino at %s!", serialPort)


############################ ROS get joystick input ###########################

def callback(joyPub):
    try:
        ser.write(serHeader + chr(joyPub.axisInputs[0]) + chr(joyPub.axisInputs[1]) + chr(joyPub.axisInputs[2]) + chr(joyPub.axisInputs[3]))
        if verboseOn: rospy.logerr("A0: %s   A1: %s   A2: %s   A3: %s", joyPub.axisInputs[0], joyPub.axisInputs[1], joyPub.axisInputs[2], joyPub.axisInputs[3])
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")
    rospy.sleep(dataSendInterval)


def listener():
    while not rospy.is_shutdown():
        rospy.Subscriber("tric_joy_publisher", TricJoy, callback, queue_size=1)
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
    listener()
    #mainWin.show()

