#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
from PyQt4 import QtCore, QtGui
import serial
import pygame

class HelloWindow(QtGui.QMainWindow) :
    def __init__(self, *args) :
        apply(QtGui.QMainWindow.__init__, (self,) + args)
        self.button = HelloButton(self)
        self.setCentralWidget(self.button)
        self.connect(self.button, QtCore.SIGNAL("clicked()"),
                     self, QtCore.SLOT("close()"))

class HelloButton(QtGui.QPushButton) :
    def __init__(self, *args) :
        apply(QtGui.QPushButton.__init__, (self,) + args)
        self.setText("Hello, World!")

class HelloApp(QtGui.QApplication) :
    def __init__(self, args) :
        QtGui.QApplication.__init__(self, args)
        self.addWidgets()
        self.exec_()

    def addWidgets(self) :
        self.hellobutton = QtGui.QPushButton("Say 'Hello world!'", None)
        self.hellobutton.setGeometry(10, 10, 60, 35)
        self.connect(self.hellobutton, QtGui.SIGNAL("clicked()"), self.slotSayHello)
        self.hellobutton.show()

    def slotSayHello(self) :
        print "Hello world!"


class BoxLayout(QtGui.QWidget) :
    def __init__(self, parent=None) :
        QtGui.QWidget.__init__(self, parent)
        self.setWindowTitle('box layout')

        ok = QtGui.QPushButton("OK")
        cancel = QtGui.QPushButton("Cancel")
        
        hbox = QtGui.QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(ok)
        hbox.addWidget(cancel)

        vbox = QtGui.QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(hbox)

        self.setLayout(vbox)
        self.resize(300, 150)

def main(args) :
    app = QtGui.QApplication(args)
    win = HelloWindow()
    win.show()
    app.connect(app, QtCore.SIGNAL("lastWindowClosed()"),
                app, QtCore.SLOT("quit()"))
    app.exec_()



if __name__ == "__main__" :
    joyInit()
    joyGetEvent()
