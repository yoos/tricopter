#! /usr/bin/env python
# -*- coding: utf-8 -*-

from Tkinter import *
import serial
import pygame

root = Tk()

ardPort = "/dev/ttyUSB0"
header = chr(255)
dogBone = chr(254) # Feed the watchdog

try:
    ser = serial.Serial(ardPort, 4800)
    print "Arduino at %s" % ardPort
except:
    print "No arduino!"

joy = []
msg = ""
num = ""
updatedAxis = 0
axisValue = 128
vecR = 0
vecX = 0
vecY = 0

def key(event):
    """Describe event"""
    event_name = {'2': 'KeyPress', '4': 'ButtonPress'}
    someApp.labela.config(text=event.char)
    print "Time:", str(event.time)
    print "EventType=" + str(event.type), \
        event_name[str(event.type)], \
        "EventWidgetId=" + str(event.widget), \
        "EventKeySymbol=" + str(event.keysym)
    if event.char == "q":
        quit()
    if event.keysym == "Left":
        quit()
        

def sendData():
    try:
        ser.write(header + chr(updatedAxis) + chr(axisValue) + chr(axisValue) + chr(updatedAxis)) # Redundancy. After a little thought, I concluded that this would preclude errors most completely.
    except:
        print "sendData failed"

def feedDog():
    try:
        ser.write(dogBone) # Feed the watchdog
    except:
        pass

class App:
    def __init__(self, master):
        frame = Frame(master, width=800, height=500)
        frame.pack()

        hellobye = Frame(frame, width=300, height=200)
        hellobye.pack(side=BOTTOM)

        self.labela = Label(frame)
        self.labela.pack(side=LEFT)

        self.labelv = Label(frame)
        self.labelv.pack(side=RIGHT)

        self.button = Button(hellobye, text="Quit", command=frame.quit)
        self.button.pack(side=LEFT)

        self.hello = Button(hellobye, text="Hello", command=self.say_hello)
        self.hello.pack(side=RIGHT)

    def say_hello(self):
        print "Hello world!"

def startJoy():
    pygame.joystick.init()
    pygame.display.init()

    if not pygame.joystick.get_count():
        print "No joystick."
        quit()

    stick = pygame.joystick.Joystick(0)
    stick.init()
    print "Joystick:", stick.get_name()

def getJoyEvents():
    event = pygame.event.poll()
    feedDog()
    if (event.type == pygame.JOYAXISMOTION):
        global updatedAxis, axisValue
        updatedAxis = event.dict['axis']
        axisValue = int((event.dict['value']+1)/2 * 255 + 0.5)
        if updatedAxis == 1:
            axisValue = 255 - axisValue # Y axis is flipped on my accelerometer

        someApp.labela.config(text=updatedAxis)
        someApp.labelv.config(text=axisValue)

        print "Axis %d: %d" % (updatedAxis, axisValue)

        sendData()

    root.after(10, getJoyEvents)


someApp = App(root)


if __name__ == "__main__":
    root.bind_all("<Key>", key)
    root.after(100, startJoy)
    root.after(1000, getJoyEvents)
    root.after(1050, sendData)
    root.mainloop()


