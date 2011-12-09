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

# Comm values
#serialPort = "/dev/ttyUSB0"   # Uncomment to specify serial port. Otherwise, will connect to first available port.
baudRate = 57600
dataSendInterval = 0.030   # 30 ms interval = 33.3 Hz. NOTE: This frequency should be LOWER than the microcontroller's control loop frequency!
dogFeedInterval = 0.1
serHeader = chr(255)
dogBone = chr(254)
verboseOn = True

armed = False
printString = ''

axisSigns = [-1, 1, 1, 1, -1, 1]   # Axis sign flips
axisX = 0
axisY = 1
axisT = 2
axisZ = 3

axisValues = [126, 126, 126, 3]   # Keep Z value at some non-zero value (albeit very low so the tricopter doesn't fly off if something goes awry) so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
buttonValues = 0   # Bitfield.

rospy.init_node("tric_comm", anonymous=True)

# =============================================================================
# Try to initialize a serial connection. If serialPort is defined, try opening
# that. If it is not defined, loop through a range of integers starting from 0
# and try to connect to /dev/ttyUSBX where X is the integer. In either case,
# process dies if serial port cannot be opened.
# =============================================================================
try:
    ser = serial.Serial(serialPort, baudRate, timeout=0)
except serial.SerialException:
    rospy.logerr("Unable to open specified serial port!")
    exit(1)
except NameError:
    for i in range(4):
        try:
            ser = serial.Serial("/dev/ttyUSB"+str(i), baudRate, timeout=0)
            rospy.loginfo("Opened serial at /dev/ttyUSB%d.", i)
            break
        except serial.SerialException:
            rospy.logerr("No serial at /dev/ttyUSB%d.", i)
            if i == 3:
                rospy.logerr("No serial found. Giving up!")
                exit(1)


############################ ROS get joystick input ###########################

# Convert joystick values of [-1.0, 1.0] to [1, 251] so we can send them over
# serial as bytes.
def joy2int(joyVal, axisIndex):
    return int(250*((axisSigns[axisIndex] * joyVal + 1) / 2) + 1)

# Update axisValues when joystick is updated.
def callback(myJoy):
    global axisValues, buttonValues
    """
        Raw axis values [-1, 1] for X, Y, and T axes are cubed (to facilitate
        fine control before being scaled and mapped to [0, 1]. All of this is
        then mapped to [0, 250], which is finally shifted to [1, 251] to be
        sent as bytes.
    """
    axisValues[axisX] = joy2int(myJoy.axes[0], axisX)   # X
    axisValues[axisY] = joy2int(myJoy.axes[1], axisY)   # Y

    if len(myJoy.axes) == 3:
        axisValues[axisT] = joy2int(0, axisT)   # Dummy T
        axisValues[axisZ] = joy2int(myJoy.axes[2], axisZ)   # Z
    else:
        axisValues[axisT] = joy2int(myJoy.axes[2], axisT)   # T
        axisValues[axisZ] = joy2int(myJoy.axes[3], axisZ)   # Z

        # Joystick at OSURC
        #axisValues[axisT] = joy2int(myJoy.axes[3], axisT)   # T
        #axisValues[axisZ] = joy2int(myJoy.axes[2], axisZ)   # Z

    buttonValues = 0
    for i in range(len(myJoy.buttons)):
        buttonValues += myJoy.buttons[i]<<i


# Send axisValues to tricopter.
def communicate():
    global armed
    global printString
    if armed:
        sendData(serHeader + chr(axisValues[axisX]) + chr(axisValues[axisY]) + chr(axisValues[axisT]) + chr(axisValues[axisZ]) + chr(buttonValues & 0b01111111) + chr(buttonValues >> 7))
        # sendData(serHeader + chr(126) + chr(126) + chr(126) + chr(74))
        rospy.loginfo(str(axisValues[axisX]) + " " + str(axisValues[axisY]) + " " + str(axisValues[axisT]) + " " + str(axisValues[axisZ]) + " " + str(buttonValues))
    elif not armed:
        rospy.loginfo("Current throttle value: " + str(axisValues[axisZ]))
        if axisValues[axisZ] == 1:   # If throttle is at minimum position
            armed = True
            rospy.loginfo("Joystick throttle at minimum! Motors armed!")
    # readData()   # TODO: make this work.
    
# Serial write.
def sendData(myStr):
    try:
        # ser.write(myStr)
        for i in range(len(myStr)):
            ser.write(myStr[i])
            rospy.sleep(0.0001)   # Sometimes, a delay seems to help. Maybe?
    except:
        if verboseOn: rospy.logerr("ERROR: Unable to send data. Check connection.")
        # TODO: Comm should do something to ensure safety when it loses connection.

# DEPRECATED: Serial read.
#def readData():
#    global printString
#    try:
#        while ser.inWaiting() > 0:
#            RX = ser.readline()   # TODO: Try changing this to ser.read(1)?
#        printString += RX
#    except:
#        return 0

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
            #rospy.loginfo(self.times)
            #rospy.loginfo(printString)
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

