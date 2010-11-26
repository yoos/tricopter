#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest("tricopter")
import sys
import rospy
from std_msgs.msg import String
import threading
from PyQt4 import Qt
import serial
import pygame

ardport = "/dev/ttyUSB0"
dogbone = chr(255) # Feed watchdog

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

def init():
    # Find Arduino
    try:
        ser = Serial(ardport, 4800)
        print "Arduino at %s" % ardport
    except:
        print "No Arduino!"

    # Find joystick
    pygame.joystick.init()
    pygame.display.init()
    try:
        stick = pygame.joystick.Joystick(0)
        stick.init()
        print "Joystick:", stick.get_name()
    except:
        print "No joysticks found. Aborting..."

def sendData(*args):
    try:
        ser.write(args)
    except:
        print "sendData failed"

def feedDog():
    try:
        ser.write(dogbone)
    except:
        pass


def joyGetEvent():
    event = pygame.event.poll()
    feedDog()

    if event.type == pygame.JOYAXISMOTION:
        global newaxis, newaxisvalue
        newaxis = event.dict['axis']
        newaxisvalue = int((event.dict['value']+1)/2 * 255 + 0.5) # Document what this exactly does
        print "Axis %d: %d" % (newaxis, newaxisvalue)
        sendData()

    t = threading.Timer(0.02, joyGetEvent) # 50 Hz
    t.start()



class MainWindow(Qt.QMainWindow):
    def __init__(self, *args):
        apply(Qt.QMainWindow.__init__, (self,) + args)

class CtrlButton(Qt.QPushButton):
    def __init__(self, *args):
        apply(Qt.QPushButton.__init__, (self,) + args)
        self.connect(self, SIGNAL("clicked()"), self.doPrint)
    def doPrint(self):
        print "Control button clicked"



#mainwin = MainWindow()

if __name__ == "__main__" :
    init()
    joyGetEvent()
    listener()
    #mainwin.show()

