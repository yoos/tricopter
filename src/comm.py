#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import Qt
import serial
import threading
from threading import Timer, Thread
from signal import signal, SIGINT

# ROS stuff
import roslib; roslib.load_manifest("tricopter")
import rospy
from joy.msg import Joy
from tricopter.msg import TricJoy

# Comm values
serialPort = "/dev/ttyUSB0"
baudRate = 57600
dataSendInterval = 0.05   # Interval between data sends in seconds
dogFeedInterval = 0.1
serHeader = chr(255)
dogBone = chr(254)
verboseOn = True

armed = False

axisSigns = [-1, 1, -1, 1, -1, 1]   # Axis sign flips
axisValues = [126, 126, 126, 42]   # Keep Z value at some non-zero value so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
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
    axisValues[0] = int(250*(axisSigns[0] * myJoy.axes[0] + 1) / 2 + 1)   # Range 1-251 in order to send as char value
    axisValues[1] = int(250*(axisSigns[1] * myJoy.axes[1] + 1) / 2 + 1)
    axisValues[2] = int(250*(axisSigns[2] * myJoy.axes[2] + 1) / 2 + 1)
    axisValues[3] = int(250*(axisSigns[3] * myJoy.axes[3] + 1) / 2 + 1)

    if armed:
        sendData(serHeader + chr(axisValues[0]) + chr(axisValues[1]) + chr(axisValues[2]) + chr(axisValues[3]))
        if verboseOn: rospy.logerr("A0: %s   A1: %s   A2: %s   A3: %s", axisValues[0], axisValues[1], axisValues[2], axisValues[3])
    elif not armed:
        rospy.logerr("Move joystick throttle to minimum position in order to send motor arming signal.")
        if axisValues[3] == 1:   # If throttle is at minimum position
            armed = True
            rospy.logerr("Joystick throttle at minimum! Motors armed!")
    
    rospy.sleep(dataSendInterval)

def sendData(myStr):
    try:
        ser.write(myStr)
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")

class TricSubscriber(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            rospy.Subscriber("joy", Joy, callback, queue_size=1)
            rospy.spin()

class TricWatchdog(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.times = 0
    def run(self):
        while self.running and not rospy.is_shutdown():
            self.times += 1
            sendData(dogBone)
            rospy.loginfo(self.times)
            rospy.sleep(dogFeedInterval)
            

#################################### Qt GUI ###################################

#class MainWindow(Qt.QMainWindow):
#    def __init__(self, *args):
#        apply(Qt.QMainWindow.__init__, (self,) + args)
#
#class CtrlButton(Qt.QPushButton):
#    def __init__(self, *args):
#        apply(Qt.QPushButton.__init__, (self,) + args)
#        self.connect(self, SIGNAL("clicked()"), self.doPrint)
#    def doPrint(self):
#        rospy.logerr("Control button clicked")

#mainWin = MainWindow()

#################################### Main #####################################

if __name__ == "__main__" :
    try:
        tricSub = TricSubscriber()
        tricSub.start()   # Start subscriber thread.
        tricWatch = TricWatchdog()
        tricWatch.start()   # Start watchdog thread.
        raw_input("Hit <enter> to quit")

        # Stop the while loops
        tricSub.running = False
        tricWatch.running = False

        # Wait for threads to finish jobs
        tricSub.join()
        tricWatch.join()

    except rospy.ROSInterruptException:
        pass
    #mainWin.show()

