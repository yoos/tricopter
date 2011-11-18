#!/usr/bin/env python

###############################################################################
# serialmon.py
###############################################################################

import sys
import serial
import array
import string
import struct
from math import sqrt
from time import sleep
import threading
from threading import Timer, Thread
from signal import signal, SIGINT

# ROS stuff
import roslib; roslib.load_manifest("tricopter")
import rospy

try:
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
    print "OpenGL successfully imported."
except:
    print "Error: PyOpenGL not installed properly. Exiting..."
    sys.exit()

# Initialize ROS node.
rospy.init_node("tric_vis", anonymous=True)


# Number of the glut window.
window = 0

# Initial DCM values. Initialize these separately (i.e., don't do dcm = dcmT = [...]), otherwise the DCM values will be read in an incorrect order!
dcm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
dcmT = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

# Target rotation values
targetRot = [0.0, 0.0, 0.0]

# Motor/servo values (MT, MR, ML, ST)
motorVal = [0.0, 0.0, 0.0, 0.0]


def drawScene():
    # Define axes to draw.
    axes = dcm

    # Initial vertex values for a box drawn around the DCM.
    dcmBox = [[-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1], [-1,-1,-1], [-1,1,-1], [1,1,-1], [1,-1,-1]]
    
    # Initial vertex values for the three motor positions (tail, right, left).
    motorBase = [[0, -1, 0], [sqrt(3)/2, 1/2, 0], [-sqrt(3)/2, 1/2, 0]]   # The "base" of the motors.
    motorTop = [[0, -1, 0.1], [sqrt(3)/2, 1/2, 0.1], [-sqrt(3)/2, 1/2, 0.1]]   # The "top" of the motors.
    motorColor = [[1,1,1], [1,1,1], [1,1,1]]

    # Clear screen and depth buffer.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Reset view
    glLoadIdentity()

    # Move object into screen so it's not in my face (i.e,. invisible).
    glTranslatef(0.0, 0.0, -3.0)

    # Syntax: glRotatef(angle, x, y, z)
    glRotatef(-90.0, 1.0, 0.0, 0.0)
    #glRotatef(-20.0, 0.0, 1.0, 0.0)

    ###########################################################################
    # DCM visualization
    ###########################################################################

    # Calculate vertex locations for a box. Refer to the declaration of dcmBox to
    # see the order of the vertices.
    for i in range(3):
        dcmBox[0][i] = (-axes[0][i] -axes[1][i] +axes[2][i]) * 0.4
        dcmBox[1][i] = (-axes[0][i] +axes[1][i] +axes[2][i]) * 0.4
        dcmBox[2][i] = ( axes[0][i] +axes[1][i] +axes[2][i]) * 0.4
        dcmBox[3][i] = ( axes[0][i] -axes[1][i] +axes[2][i]) * 0.4
        dcmBox[4][i] = (-axes[0][i] -axes[1][i] -axes[2][i]) * 0.4
        dcmBox[5][i] = (-axes[0][i] +axes[1][i] -axes[2][i]) * 0.4
        dcmBox[6][i] = ( axes[0][i] +axes[1][i] -axes[2][i]) * 0.4
        dcmBox[7][i] = ( axes[0][i] -axes[1][i] -axes[2][i]) * 0.4

    # Draw the axes of whichever DCM we're using.
    glBegin(GL_LINES)
    glColor3f(1,0,0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[0])
    glColor3f(0,1,0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[1])
    glColor3f(0,0,1)
    glVertex3f(0.0, 0.0, 0.0); glVertex3fv(axes[2])
    glEnd()

    # Draw a box around the DCM to help visualize.
    glBegin(GL_LINES)
    glColor3f(1,1,1)
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[0]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[2]); glVertex3fv(dcmBox[6])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[1])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[5]); glVertex3fv(dcmBox[6])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[3])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[4])
    glVertex3fv(dcmBox[7]); glVertex3fv(dcmBox[6])
    glEnd()

    # Draw static axes.
    glBegin(GL_LINES)
    glColor3f(0.2, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(1.5, 0.0, 0.0)
    glColor3f(0.0, 0.2, 0.0)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 1.5, 0.0)
    glColor3f(0.0, 0.0, 0.2)
    glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 1.5)
    glColor3f(0.2, 0.2, 0.2)
    glEnd()

    # Draw a static box.
    #glutWireCube(1.2)


    ###########################################################################
    # Motor output visualization
    ###########################################################################

    # Motor base locations
    for i in range(3):
        motorBase[0][i] = -axes[1][i]                            # Tail (0, -1, 0)
        motorBase[1][i] = axes[0][i]*sqrt(3)/2 + axes[1][i]/2    # Right (sqrt(3)/2, 1/2, 0)
        motorBase[2][i] = -axes[0][i]*sqrt(3)/2 + axes[1][i]/2   # Left (-sqrt(3)/2, 1/2, 0)

    # Motor "height" to make them visible. TODO: these should eventually be cylinders.
    for i in range(3):
        motorTop[0][i] = -axes[1][i] + axes[2][i]*motorVal[0]/125                           # Tail (0, -1, motorVal)
        motorTop[1][i] = axes[0][i]*sqrt(3)/2 + axes[1][i]/2 + axes[2][i]*motorVal[1]/125   # Right (sqrt(3)/2, 1/2, motorVal)
        motorTop[2][i] = -axes[0][i]*sqrt(3)/2 + axes[1][i]/2 + axes[2][i]*motorVal[2]/125  # Left (-sqrt(3)/2, 1/2, motorVal)

    # Motor colors to indicate speed. TODO: this should replace changing the
    # values of motorTop.
    for i in range(3):
        if motorVal[i] < 10:
            motorColor[i] = [0.0, 0.0, 1.0]
        elif motorVal[i] >= 10 and motorVal[i] < 30:
            motorColor[i] = [0.0, (motorVal[i]-10.0)/20.0, 1.0]
        elif motorVal[i] >= 30 and motorVal[i] < 50:
            motorColor[i] = [0.0, 1.0, 1.0-(motorVal[i]-30.0)/20.0]
        elif motorVal[i] >= 50 and motorVal[i] < 70:
            motorColor[i] = [(motorVal[i]-50.0)/20.0, 1.0, 0.0]
        elif motorVal[i] >= 70 and motorVal[i] < 90:
            motorColor[i] = [1.0, 1.0-(motorVal[i]-70.0)/20.0, 0]
        else:
            motorColor[i] = [1.0, 0.0, 0.0]

    glBegin(GL_LINES)
    glColor3fv(motorColor[0])
    glVertex3fv(motorBase[0]); glVertex3fv(motorTop[0])
    glColor3fv(motorColor[1])
    glVertex3fv(motorBase[1]); glVertex3fv(motorTop[1])
    glColor3fv(motorColor[2])
    glVertex3fv(motorBase[2]); glVertex3fv(motorTop[2])
    glEnd()

    # Since this is double buffered, swap the buffers to display what just got drawn.
    glutSwapBuffers()


def resizeScene(width, height):
    # Protect against divide by zero when window size is small.
    if height == 0:
        height = 1

    # Reset current viewport and perspective transformation.
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)


def initGL(width, height):
    glClearColor(0.0, 0.0, 0.0, 0.0)   # Clear background color to black.
    glClearDepth(1.0)                  # Enable clearing of the depth buffer.
    glDepthFunc(GL_LESS)               # Type of depth test.
    glEnable(GL_DEPTH_TEST)            # Enable depth testing.
    glShadeModel(GL_SMOOTH)            # Enable smooth color shading.

    glMatrixMode(GL_PROJECTION)   # Specify which matrix is the current matrix.
    glLoadIdentity()              # Reset the projection matrix.

    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)


class visualizationThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        global window
        while self.running and not rospy.is_shutdown():
            glutInit(sys.argv)

            glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
            glutInitWindowSize(640, 480)
            glutInitWindowPosition(0, 0)

            # Initialize window so we can close it later.
            window = glutCreateWindow("IMU visualization")

            # Register the drawing function with glut.
            glutDisplayFunc(drawScene)

            # When doing nothing, redraw scene.
            glutIdleFunc(drawScene)

            # Register the function called when window is resized.
            glutReshapeFunc(resizeScene)

            # Register the function called when key is pressed.
            #glutKeyboardFunc(keyPressed)

            # Initialize window.
            initGL(640, 480)

            # Start event processing engine.
            glutMainLoop()


class telemetryThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        global dcm, dcmT, targetRot, motorVal
        dcmDataIndex = -1       # Do I see IMU data?
        rotDataIndex = -1       # Do I see rotation data?
        motorDataIndex = -1     # Do I see motor data?
        serBuffer = ''
        serLines = ''

        try:
            ser = serial.Serial("/dev/ttyUSB1", 57600)
        except:
            print "Serial unavailable!"

        while self.running and not rospy.is_shutdown():
            try:
                if ser.inWaiting() > 0:
                    # Update buffer, adding onto incomplete line if necessary.
                    serBuffer = serBuffer + ser.read(ser.inWaiting())

                    # Check for separator tag 0xdeadbeef and split one entry off buffer.
                    if '\xde\xad\xbe\xef' in serBuffer:
                        serLines = serBuffer.split('\xde\xad\xbe\xef')

                        # Parse fields separated by 0xf0f0.
                        fields = serLines[-2].split('\xff\xff')

                        # Save second to last line and discard rest.
                        serBuffer = serLines[-1]

                    # Check if we're receiving DCM data.
                    if dcmDataIndex == -1:
                        for i in range(len(fields)):
                            if 'DCM' in fields[i]:
                                dcmDataIndex = i
                    else:
                        # Structure of DCM block:
                        #     'DCMxxxxxxxxx', where x represents a single byte.
                        #     The 9 floats of the DCM are mapped to one-byte
                        #     integers before being pushed to serial in the
                        #     same scheme in which we encode joystick values in
                        #     comm.py.
                        try:
                            for i in range(3):
                                for j in range(3):
                                    dcm[i][j] = float(int(fields[dcmDataIndex][i*3+j+3:i*3+j+4].encode('hex'), 16)-1)/250*2-1
                                    #dcm[i][j] = struct.unpack('f', fields[dcmDataIndex][3+(i*3+j)*4:3+(i*3+j)*4+4])[0]
                                    dcmT[j][i] = dcm[i][j]
                        except Exception, e:
                            print "DCM: " + str(e)

                    # Check if we're receiving target rotation data.
                    if rotDataIndex == -1:
                        for i in range(len(fields)):
                            if 'ROT' in fields[i]:
                                rotDataIndex = i
                    else:
                        for i in range(3):
                            targetRot[i] = struct.unpack('f', fields[rotDataIndex][3+i*4:3+i*4+4])[0]

                    # Check if we're receiving motor/servo output data.
                    if motorDataIndex == -1:
                        for i in range(len(fields)):
                            if 'MTR' in fields[i]:
                                motorDataIndex = i
                    else:
                        try:
                            for i in range(4):
                                motorVal[i] = int(fields[motorDataIndex][i+3:i+4].encode('hex'), 16)
                                #motorVal[i] = struct.unpack('f', fields[motorDataIndex][3+i*4:3+(i+1)*4])[0]
                        except Exception, e:
                            print "MTR: " + str(e)

                    print fields
                    #print [dcm, fields[-1]]
                    #print [targetRot, fields[-1]]
                    print [int(fields[0].encode('hex'), 16), motorVal, fields[-1]]

            except:
                pass
            rospy.sleep(0.005)


###############################################################################

if __name__ == "__main__":
    try:
        telemetry = telemetryThread()
        telemetry.start()
        vis = visualizationThread()
        vis.start()
        raw_input("Hit <enter> to quit.")

        # Stop the loops.
        telemetry.running = False
        vis.running = False

        # Wait for threads to finish jobs.
        telemetry.join()
        vis.join()

    except rospy.ROSInterruptException:
        pass

