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
baudRate = 57600
inputRate = 400   # Inpt rate in Hz
serHeader = chr(255)
okayToSend = False
verboseOn = True

rospy.init_node("tric_listener", anonymous=False)

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate)
    rospy.loginfo("Arduino at %s", serialPort)
except:
    rospy.logerr("No Arduino at %s!", serialPort)


############################ ROS get joystick input ###########################

def callback(joyPub):

    sendData()
        
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
        rospy.Subscriber("tric_joy_publisher", String, callback)
        if okayToSend:
            sendData()
        #rate.sleep()
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

