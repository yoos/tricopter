#!/usr/bin/env python

import sys
import serial
import array
import string
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

# Initial DCM values.
dcm = dcmT = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

# Initial vertex values for a box drawn around the DCM.
vx = [[-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1], [-1,-1,-1], [-1,1,-1], [1,1,-1], [1,-1,-1]]


#def vCross (v1[3], v2[3], vOut[3]):
#    vOut[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
#    vOut[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
#    vOut[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);

# Multiply two 3x3 matrices.
#def mProduct (m1, m2, mOut):
#    tmp = range(3)
#    for i in range(3):
#        for j in range(3):
#            for k in range(3):
#                tmp[k] = m1[i][k] * m2[k][j];
#            mOut[i][j] = sum(tmp)


def drawScene():
    # Define axes to draw.
    axes = dcmT

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)   # Clear screen and depth buffer.
    glLoadIdentity()      # Reset view
    #glLoadMatrix(glDCM)   # Replace current matrix with glDCM, a 4x4 matrix.

    # Move object into screen so it's not in my face (i.e,. invisible).
    glTranslatef(0.0, 0.0, -3.0)

    # Calculate vertex locations for a box. Refer to the declaration of vx to
    # see the order of the vertices.
    for i in range(3):
        vx[0][i] = (-axes[0][i] -axes[1][i] +axes[2][i]) * 0.4
        vx[1][i] = (-axes[0][i] +axes[1][i] +axes[2][i]) * 0.4
        vx[2][i] = ( axes[0][i] +axes[1][i] +axes[2][i]) * 0.4
        vx[3][i] = ( axes[0][i] -axes[1][i] +axes[2][i]) * 0.4
        vx[4][i] = (-axes[0][i] -axes[1][i] -axes[2][i]) * 0.4
        vx[5][i] = (-axes[0][i] +axes[1][i] -axes[2][i]) * 0.4
        vx[6][i] = ( axes[0][i] +axes[1][i] -axes[2][i]) * 0.4
        vx[7][i] = ( axes[0][i] -axes[1][i] -axes[2][i]) * 0.4

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
    glVertex3fv(vx[0]); glVertex3fv(vx[1])
    glVertex3fv(vx[0]); glVertex3fv(vx[3])
    glVertex3fv(vx[0]); glVertex3fv(vx[4])
    glVertex3fv(vx[2]); glVertex3fv(vx[1])
    glVertex3fv(vx[2]); glVertex3fv(vx[3])
    glVertex3fv(vx[2]); glVertex3fv(vx[6])
    glVertex3fv(vx[5]); glVertex3fv(vx[1])
    glVertex3fv(vx[5]); glVertex3fv(vx[4])
    glVertex3fv(vx[5]); glVertex3fv(vx[6])
    glVertex3fv(vx[7]); glVertex3fv(vx[3])
    glVertex3fv(vx[7]); glVertex3fv(vx[4])
    glVertex3fv(vx[7]); glVertex3fv(vx[6])
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
    glutWireCube(1.2)

    # Since this is double buffered, swap the buffers to display what just got drawn.
    glutSwapBuffers()


def resizeScene(width, height):
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
    global window
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            glutInit(sys.argv)

            glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
            glutInitWindowSize(640, 480)
            glutInitWindowPosition(0, 0)

            # Initialize window so we can close it later.
            window = glutCreateWindow("IMU visualization")

            # Register the drawing function with glut
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
        global dcm, dcmT
        serialIsClean = False
        imuDataIndex = -1   # Do I see IMU data in datafeed?
        line = ''
        try:
            ser = serial.Serial("/dev/ttyUSB0", 57600)
        except:
            print "Serial unavailable!"

        while self.running and not rospy.is_shutdown():
            # TODO: get serial input as raw bits.
            try:
                # Clean up beginning of serial so we can read a full line.
                while not serialIsClean:
                    if ser.read(1) == '\n':
                        serialIsClean = True

                # Read from serial.
                if ser.inWaiting() > 0:
                    # Read one line ending with \n.
                    line = ser.readline()

                    # Parse fields separated by spaces.
                    fields = line[:-1].split(' ')

                    if imuDataIndex == -1:
                        for i in range(len(fields)):
                            if fields[i] == 'iD':
                                print fields[i]
                                imuDataIndex = i

                    if imuDataIndex != -1:
                        for i in range(3):
                            for j in range(3):
                                dcm[i][j] = float(fields[imuDataIndex+i*3+j+1])
                        for i in range(3):
                            for j in range(3):
                                dcmT[i][j] = dcm[j][i]
                        print dcm
                    else:
                        print "No IMU"

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

