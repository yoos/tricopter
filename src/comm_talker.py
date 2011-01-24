#!/usr/bin/env python
import roslib; roslib.load_manifest("tricopter")
import rospy
from joy.msg import Joy
from tricopter.msg import TricJoy

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
axisValue = [126, 126, 126, 126]   # Keep Z value at non-zero so user is forced to fiddle with throttle before motors arm. Hopefully prevents disasters.


pub = rospy.Publisher("tric_joy_publisher", TricJoy)

def callback(myJoy):
    global axisValue

    # Calculate axis values.
    axisValue[0] = int(250*(xSign * myJoy.axes[xAxis] + 1) / 2 + 1)   # Range 1-251 in order to send as char value
    axisValue[1] = int(250*(ySign * myJoy.axes[yAxis] + 1) / 2 + 1)
    axisValue[2] = int(250*(tSign * myJoy.axes[tAxis] + 1) / 2 + 1)
    axisValue[3] = int(250*(zSign * myJoy.axes[zAxis] + 1) / 2 + 1)

    pub.publish(axisValue)

def tric_joy_publisher():
    rospy.init_node("tric_joy_publisher")

    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        tric_joy_publisher()
    except rospy.ROSInterruptException: pass

