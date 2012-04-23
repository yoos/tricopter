#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
import threading
from threading import Timer, Thread
from signal import signal, SIGINT

# ROS
import roslib; roslib.load_manifest("tricopter")
import rospy
from sensor_msgs.msg import Joy

# Tricopter
import triconfig as cfg   # Import config.

armed = False   # System arm status. Set to True once throttle is set to zero. Communication will not start until this is True.

axisValues = [125, 125, 125, 3]   # [X, Y, T, Z] -- Initialize Z as some non-zero value (albeit very low so the tricopter doesn't fly off if something goes awry) so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
buttonValues = 0   # Bitfield.


# =============================================================================
# Process joystick input.
# =============================================================================

# Convert joystick values of [-1.0, 1.0] to [0, 250] so we can send them over
# serial as bytes.
def joy2byte (joyVal, axisIndex):
    return int(250 * ((cfg.axisSigns[axisIndex] * joyVal + 1) / 2))

# Update axisValues when joystick is updated. Raw axis values [-1, 1] for X, Y,
# and T axes are cubed to facilitate fine control before being scaled and
# mapped to [0, 1]. All of this is then mapped to [0, 250] to be sent as bytes.
def joyCallback (myJoy):
    global axisValues, buttonValues

    axisValues[cfg.axisX] = joy2byte(myJoy.axes[0], cfg.axisX)   # X
    axisValues[cfg.axisY] = joy2byte(myJoy.axes[1], cfg.axisY)   # Y

    if len(myJoy.axes) == 3:
        axisValues[cfg.axisT] = joy2byte(0, cfg.axisT)   # Dummy T
        axisValues[cfg.axisZ] = joy2byte(myJoy.axes[2], cfg.axisZ)   # Z
    else:
        axisValues[cfg.axisT] = joy2byte(myJoy.axes[2], cfg.axisT)   # T
        axisValues[cfg.axisZ] = joy2byte(myJoy.axes[3], cfg.axisZ)   # Z

        # Mars Rover joystick
        #axisValues[cfg.axisT] = joy2byte(myJoy.axes[3], cfg.axisT)   # T
        #axisValues[cfg.axisZ] = joy2byte(myJoy.axes[2], cfg.axisZ)   # Z

    buttonValues = 0
    for i in range(len(myJoy.buttons)):
        buttonValues += myJoy.buttons[i]<<i


# =============================================================================
# Communicate.
# =============================================================================
def transmit():
    global armed

    if armed:
        serWrite(cfg.serHeader +
                 chr(axisValues[cfg.axisX]) +
                 chr(axisValues[cfg.axisY]) +
                 chr(axisValues[cfg.axisT]) +
                 chr(axisValues[cfg.axisZ]) +
                 chr(buttonValues & 0b01111111) +
                 chr(buttonValues >> 7))
        rospy.loginfo(str(axisValues[cfg.axisX]) + " " +
                      str(axisValues[cfg.axisY]) + " " +
                      str(axisValues[cfg.axisT]) + " " +
                      str(axisValues[cfg.axisZ]) + " " +
                      str(buttonValues))
    elif not armed:
        rospy.loginfo("[GS] Current throttle value: " + str(axisValues[cfg.axisZ]))
        if axisValues[cfg.axisZ] == 0:   # If throttle is at minimum position
            armed = True
            rospy.loginfo("[GS] Joystick throttle at minimum! Initialized communication!")

# Serial write.
def serWrite(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
            rospy.sleep(0.00005)   # Sometimes, a delay seems to help. Maybe?
    except:
        rospy.logerr("[GS] Unable to send data. Check connection.")
        # TODO: Comm should do something to ensure safety when it loses connection.


# =============================================================================
# Threads
# =============================================================================
class TriSubscriber(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            rospy.Subscriber("joy", Joy, joyCallback, queue_size=1)
            rospy.spin()

class TriTX(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.times = 0
    def run(self):
        while self.running and not rospy.is_shutdown():
            transmit()
            self.times += 1
            rospy.sleep(cfg.dataSendInterval)

class TriWatchdog(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.times = 0
    def run(self):
        while self.running and not rospy.is_shutdown():
            serWrite(cfg.dogBone)
            self.times += 1
            rospy.sleep(cfg.dogFeedInterval)


###############################################################################

if __name__ == "__main__":
    rospy.init_node("tricopter_ground_station", anonymous=False)

    # =========================================================================
    # Try to initialize a serial connection. If serialPort is defined, try
    # opening that. If it is not defined, loop through a range of integers
    # starting from 0 and try to connect to /dev/ttyUSBX where X is the
    # integer. In either case, process dies if serial port cannot be opened.
    #
    # TODO: This needs to be made more concise.
    # =========================================================================
    try:
        ser = serial.Serial(cfg.serialPort, cfg.baudRate, timeout=0)
    except serial.SerialException:
        rospy.logerr("[GS] Unable to open specified serial port! Exiting...")
        exit(1)
    except AttributeError:
        for i in range(4):
            try:
                ser = serial.Serial("/dev/ttyUSB"+str(i), cfg.baudRate, timeout=0)
                rospy.loginfo("[GS] Opened serial port at /dev/ttyUSB%d.", i)
                break
            except serial.SerialException:
                rospy.logerr("[GS] No serial at /dev/ttyUSB%d.", i)
                if i == 3:
                    rospy.logerr("[GS] No serial found. Giving up!")
                    exit(1)

    try:
        sub = TriSubscriber()
        sub.start()
        tx = TriTX()
        tx.start()
        dog = TriWatchdog()
        dog.start()
        raw_input("Hit <enter> to quit.")

        # Stop the while loops.
        sub.running = False
        tx.running = False
        dog.running = False

        # Wait for threads to finish jobs.
        sub.join()
        tx.join()
        dog.join()

    except rospy.ROSInterruptException:
        pass

