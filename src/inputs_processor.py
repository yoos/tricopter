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
    global armed, axisValues, buttonValues

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

    if armed:
        pub.publish(Inputs(axisValues, buttonValues))
    elif axisValues[cfg.axisZ] == 0:
        armed = True
        rospy.loginfo("[Inputs] Joystick throttle at minimum. Initiating communication!")
    else:
        rospy.loginfo("[Inputs] Joystick throttle not at minimum! Current value: " + str(axisValues[cfg.axisZ]))


###############################################################################

if __name__ == "__main__":
    rospy.init_node("tricopter_inputs_processor", anonymous=False)
    pub = rospy.Publisher("tricopter_inputs", Inputs)
    sub = rospy.Subscriber("joy", Joy, joyCallback, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()   # This is only here to prevent the node from dying after spawning the publisher and subscriber nodes as separate threads. That is, no spinonce() is required.

