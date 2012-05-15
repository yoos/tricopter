#!/usr/bin/env python

import sys
import serial
import threading
from threading import Timer, Thread
from signal import signal, SIGINT

# ROS
import roslib; roslib.load_manifest("tricopter")
import rospy
from sensor_msgs.msg import Joy
from tricopter.msg import Inputs

# Tricopter
import triconfig as cfg   # Import config.

# T0: inside throttle
# T1: outside throttle
# H0: horizontal hat
# H1: vertical hat
axisValues = [125, 125, 125, 3, 3, 125, 125]   # [X, Y, Z, T0, T1, H0, H1] -- Initialize (T)hrottle as some non-zero value (albeit very low so the tricopter doesn't fly off if something goes awry) so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.
buttonValues = 0   # Bitfield.


# =============================================================================
# Process joystick input.
# =============================================================================

# Convert joystick values of [-1.0, 1.0] to [0, 250] so we can send them over
# serial as bytes.
def joy2byte (joyVal, axisIndex):
    return int(250 * ((cfg.axisSigns[axisIndex] * joyVal + 1) / 2))

# Update axisValues when joystick is updated. Raw axis values [-1, 1] for X, Y,
# and Z axes are cubed to facilitate fine control before being scaled and
# mapped to [0, 1]. All of this is then mapped to [0, 250] to be sent as bytes.
def joyCallback (myJoy):
    global armed, axisValues, buttonValues

    axisValues[cfg.axisX] = joy2byte(myJoy.axes[0], cfg.axisX)   # X
    axisValues[cfg.axisY] = joy2byte(myJoy.axes[1], cfg.axisY)   # Y

    # 3-axis joystick
    if len(myJoy.axes) == 3:
        axisValues[cfg.axisZ] = joy2byte(0, cfg.axisZ)   # Dummy Z
        axisValues[cfg.axisT0] = joy2byte(myJoy.axes[2], cfg.axisT0)   # T0

    # 6-axis joystick
    elif len(myJoy.axes) == 6:
        axisValues[cfg.axisZ]  = joy2byte(myJoy.axes[2], cfg.axisZ)
        axisValues[cfg.axisT0] = joy2byte(myJoy.axes[3], cfg.axisT0)
        axisValues[cfg.axisH0] = joy2byte(myJoy.axes[4], cfg.axisH0)
        axisValues[cfg.axisH1] = joy2byte(myJoy.axes[5], cfg.axisH1)

        # Mars Rover joystick
        #axisValues[cfg.axisT] = joy2byte(myJoy.axes[3], cfg.axisT)   # T
        #axisValues[cfg.axisZ] = joy2byte(myJoy.axes[2], cfg.axisZ)   # Z

    # 7-axis joystick
    elif len(myJoy.axes) == 7:
        axisValues[cfg.axisT0] = joy2byte(myJoy.axes[2], cfg.axisT0)
        axisValues[cfg.axisZ]  = joy2byte(myJoy.axes[3], cfg.axisZ)
        axisValues[cfg.axisT1] = joy2byte(myJoy.axes[4], cfg.axisT1)
        axisValues[cfg.axisH0] = joy2byte(myJoy.axes[5], cfg.axisH0)
        axisValues[cfg.axisH1] = joy2byte(myJoy.axes[6], cfg.axisH1)

    buttonValues = 0
    for i in range(len(myJoy.buttons)):
        buttonValues += myJoy.buttons[i]<<i

    # Publish input data.
    pub.publish(Inputs(axisValues, buttonValues))


###############################################################################

if __name__ == "__main__":
    rospy.init_node("tricopter_inputs_processor", anonymous=False)
    pub = rospy.Publisher("tricopter_inputs", Inputs)
    sub = rospy.Subscriber("joy", Joy, joyCallback, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()   # This is only here to prevent the node from dying after spawning the publisher and subscriber nodes as separate threads. That is, no spinonce() is required.

