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
from sensor_msgs.msg import Joy
# from tricopter.msg import TricJoy

# Comm values
serialPort = "/dev/ttyUSB0"
baudRate = 19200
dataSendInterval = 0.025   # Interval between data sends in seconds
dogFeedInterval = 0.1
serHeader = chr(255)
dogBone = chr(254)
verboseOn = True

armed = False
printString = ''

axisSigns = [-1, 1, -1, 1, -1, 1]   # Axis sign flips
axisValues = [126, 126, 126, 3]   # Keep Z value at some non-zero value (albeit very low so the tricopter doesn't fly off if something goes awry) so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
buttonValues = []

rospy.init_node("tric_comm", anonymous=True)

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate, timeout=0)
    rospy.loginfo("Serial at %s", serialPort)
except:
    rospy.logerr("Serial unavailable at %s!", serialPort)


############################ ROS get joystick input ###########################

def callback(myJoy):
    global axisValues
    """
        Raw axis values [-1, 1] are mapped to [0, 1] then cubed to facilitate
        fine control. This is then mapped to [0, 250], which is finally shifted
        to [1, 251] to be sent as bytes.
    """
    axisValues[0] = int(250*((axisSigns[0] * myJoy.axes[0] + 1) / 2) ** 3 + 1)  # X
    axisValues[1] = int(250*((axisSigns[1] * myJoy.axes[1] + 1) / 2) ** 3 + 1)  # Y
    axisValues[2] = int(250*((axisSigns[2] * myJoy.axes[2] + 1) / 2) ** 3 + 1)  # T
    axisValues[3] = int(250*((axisSigns[3] * myJoy.axes[3] + 1) / 2) ** 3 + 1)  # Z
    rospy.loginfo("Joystick moved!")

def communicate():
    global armed
    global printString
    if armed:
        sendData(serHeader + chr(axisValues[0]) + chr(axisValues[1]) + chr(axisValues[2]) + chr(axisValues[3]))
        # sendData(serHeader + chr(126) + chr(126) + chr(126) + chr(74))
        # if verboseOn: printString += ["A0: %s   A1: %s   A2: %s   A3: %s" % (axisValues[0], axisValues[1], axisValues[2], axisValues[3])]
    elif not armed:
        rospy.loginfo("Move joystick throttle to minimum position in order to send motor arming signal.")
        if axisValues[3] == 1:   # If throttle is at minimum position
            armed = True
            rospy.loginfo("Joystick throttle at minimum! Motors armed!")
    # readData()   # TODO: make this work.
    
def sendData(myStr):
    try:
        # ser.write(myStr)
        for i in range(0, len(myStr)):
            ser.write(myStr[i])
            rospy.sleep(0.0001)   # Sometimes, a delay seems to help. Maybe?
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")

def readData():
    global printString
    try:
        while ser.inWaiting() > 0:
            RX = ser.readline()   # TODO: Try changing this to ser.read(1)?
        printString += RX
    except:
        return 0

class TricSubscriber(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            rospy.Subscriber("joy", Joy, callback, queue_size=1)
            rospy.spin()

class TricCommunicator(threading.Thread):
    global printString
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.times = 0
    def run(self):
        while self.running and not rospy.is_shutdown():
            printString = ''
            self.times += 1
            communicate()
            rospy.loginfo(self.times)
            rospy.loginfo(printString)
            rospy.sleep(dataSendInterval)

class TricWatchdog(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.times = 0
    def run(self):
        while self.running and not rospy.is_shutdown():
            self.times += 1
            sendData(dogBone)
            # rospy.loginfo(self.times)
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
#        rospy.loginfo("Control button clicked")

#mainWin = MainWindow()

#################################### Main #####################################

if __name__ == "__main__" :
    try:
        tricSub = TricSubscriber()
        tricSub.start()   # Start subscriber thread.
        tricComm = TricCommunicator()
        tricComm.start()
        tricWatch = TricWatchdog()
        tricWatch.start()   # Start watchdog thread.
        raw_input("Hit <enter> to quit")

        # Stop the while loops
        tricSub.running = False
        tricComm.running = False
        tricWatch.running = False

        # Wait for threads to finish jobs
        tricSub.join()
        tricComm.join()
        tricWatch.join()

    except rospy.ROSInterruptException:
        pass
    #mainWin.show()

