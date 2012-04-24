#!/usr/bin/env python

import sys
import threading
from threading import Timer, Thread
from math import sqrt, pi, acos

# ROS
import roslib; roslib.load_manifest("tri_vis")
import rospy
from tri_msgs.msg import Telemetry

# OpenGL
try:
    from OpenGL.GL import *
    from OpenGL.GLUT import*
    from OpenGL.GLU import *
    rospy.loginfo("[Vis] OpenGL successfully imported!")
except:
    rospy.logerr("[Vis] PyOpenGL not installed properly. Exiting...")
    exit(1)


# =============================================================================
# Telemetry data
# =============================================================================

# Initial DCM values.
dcm = [[1.0, 0.0, 0.0],
       [0.0, 1.0, 0.0],
       [0.0, 0.0, 1.0]]

# Target rotation values
#targetRot = [0.0, 0.0, 0.0]

# Motor/servo values (MT, MR, ML, ST)
motorVal = [0.0, 0.0, 0.0, 0.0]


# =============================================================================
# Subscribe to telemetry stream.
# =============================================================================
def telCallback (tel):
    dcm[0][0] = tel.dcmXX
    dcm[0][1] = tel.dcmXY
    dcm[0][2] = tel.dcmXZ
    dcm[1][0] = tel.dcmYX
    dcm[1][1] = tel.dcmYY
    dcm[1][2] = tel.dcmYZ
    dcm[2][0] = tel.dcmZX
    dcm[2][1] = tel.dcmZY
    dcm[2][2] = tel.dcmZZ

    motorVal[0] = tel.motorT
    motorVal[1] = tel.motorR
    motorVal[2] = tel.motorL
    motorVal[3] = tel.servoT


# =============================================================================
# OpenGL elements
# =============================================================================

# Number of the glut window.
window = 0

def vDot(a, b):
    return sum([a[i]*b[i] for i in range(len(a))])

def vCross(a, b):
    c = [0,0,0]
    try:
        c = [a[1]*b[2] - a[2]*b[1], \
             a[2]*b[0] - a[0]*b[2], \
             a[0]*b[1] - a[1]*b[0]]
    except:
        pass
    return c

def drawScene():
    global quadratic

    # Define axes to draw.
    axes = dcm

    # Initial vertex values for a box drawn around the DCM.
    dcmBox = [[-1,-1,1], [-1,1,1], [1,1,1], [1,-1,1], [-1,-1,-1], [-1,1,-1], [1,1,-1], [1,-1,-1]]

    # Initial vertex values for the three motor positions (tail, right, left).
    motorBase = [[0, -1, 0], [sqrt(3)/2, 1/2, 0], [-sqrt(3)/2, 1/2, 0]]   # The "base" of the motors.
    motorTop = [[0, -1, 0.1], [sqrt(3)/2, 1/2, 0.1], [-sqrt(3)/2, 1/2, 0.1]]   # The "top" of the motors.
    motorColor = [[1,1,1,0.5], [1,1,1,0.5], [1,1,1,0.5]]

    # Clear screen and depth buffer.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Reset view
    glLoadIdentity()

    # Move object into screen so it's not in my face (i.e,. invisible).
    glTranslatef(0.0, 0.0, -3.0)

    # Syntax: glRotatef(angle, x, y, z)
    glRotatef(-90.0, 1.0, 0.0, 0.0)
    #glRotatef(-20.0, 0.0, 1.0, 0.0)

    # =========================================================================
    # DCM visualization
    # =========================================================================

    # Calculate vertex locations for a box. Refer to the declaration of dcmBox to
    # see the order of the vertices.
    for i in range(3):
        dcmBox[0][i] = (-axes[0][i] -axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[1][i] = (-axes[0][i] +axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[2][i] = ( axes[0][i] +axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[3][i] = ( axes[0][i] -axes[1][i] +axes[2][i]/2) * 0.4
        dcmBox[4][i] = (-axes[0][i] -axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[5][i] = (-axes[0][i] +axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[6][i] = ( axes[0][i] +axes[1][i] -axes[2][i]/2) * 0.4
        dcmBox[7][i] = ( axes[0][i] -axes[1][i] -axes[2][i]/2) * 0.4

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

    # =========================================================================
    # Calculate the "base" and "top" locations of motors.
    # =========================================================================
    for i in range(3):
        motorBase[0][i] = -axes[1][i]                            # Tail (0, -1, 0)
        motorBase[1][i] = axes[0][i]*sqrt(3)/2 + axes[1][i]/2    # Right (sqrt(3)/2, 1/2, 0)
        motorBase[2][i] = -axes[0][i]*sqrt(3)/2 + axes[1][i]/2   # Left (-sqrt(3)/2, 1/2, 0)

    # Motor "height" to make them visible. TODO: these should eventually be cylinders.
    for i in range(3):
        motorTop[0][i] = -axes[1][i] + axes[2][i]*(motorVal[0]+0.1)/125                           # Tail (0, -1, motorVal)
        motorTop[1][i] = axes[0][i]*sqrt(3)/2 + axes[1][i]/2 + axes[2][i]*(motorVal[1]+0.1)/125   # Right (sqrt(3)/2, 1/2, motorVal)
        motorTop[2][i] = -axes[0][i]*sqrt(3)/2 + axes[1][i]/2 + axes[2][i]*(motorVal[2]+0.1)/125  # Left (-sqrt(3)/2, 1/2, motorVal)

    # =========================================================================
    # Determine color ranging from blue to green to yellow to red depending on
    # motor speed.
    # =========================================================================
    colorSteps = [10.0, 20.0, 30.0, 50.0, 70.0]
    for i in range(3):
        if motorVal[i] < colorSteps[0]:
            motorColor[i] = [0.0, 0.0, 1.0, 0.5]
        elif motorVal[i] >= colorSteps[0] and motorVal[i] < colorSteps[1]:
            motorColor[i] = [0.0, (motorVal[i]-colorSteps[0])/(colorSteps[1]-colorSteps[0]), 1.0, 0.5]
        elif motorVal[i] >= colorSteps[1] and motorVal[i] < colorSteps[2]:
            motorColor[i] = [0.0, 1.0, 1.0-(motorVal[i]-colorSteps[1])/(colorSteps[2]-colorSteps[1]), 0.5]
        elif motorVal[i] >= colorSteps[2] and motorVal[i] < colorSteps[3]:
            motorColor[i] = [(motorVal[i]-colorSteps[2])/(colorSteps[3]-colorSteps[2]), 1.0, 0.0, 0.5]
        elif motorVal[i] >= colorSteps[3] and motorVal[i] < colorSteps[4]:
            motorColor[i] = [1.0, 1.0-(motorVal[i]-colorSteps[3])/(colorSteps[4]-colorSteps[3]), 0.0, 0.5]
        else:
            motorColor[i] = [1.0, 0.0, 0.0, 0.5]

    #glBegin(GL_LINES)
    #glColor3fv(motorColor[0])
    #glVertex3fv(motorBase[0]); glVertex3fv(motorTop[0])
    #glColor3fv(motorColor[1])
    #glVertex3fv(motorBase[1]); glVertex3fv(motorTop[1])
    #glColor3fv(motorColor[2])
    #glVertex3fv(motorBase[2]); glVertex3fv(motorTop[2])
    #glEnd()

    # =========================================================================
    # Calculate the cylindrical representation of the motors.
    #
    # TODO: cylAngle never seems to really reach zero when DCM is horizontal.
    # There are math errors when the DCM flips completely over.
    # =========================================================================
    cylDef = [0.0, 0.0, 1.0]
    cylVec = [[0.0, 0.0, 0.0]]*3
    cylT   = [[0.0, 0.0, 0.0]]*3
    cylAngle = [[0.0, 0.0, 0.0]]*3
    for i in range(3):
        for j in range(3):
            cylVec[i][j] = motorTop[i][j] - motorBase[i][j]
        cylT[i] = vCross(cylDef, cylVec[i])
        # Calculate cylinder angle. Add 0.1 to motorVal to prevent division by
        # zero.
        try:
            cylAngle[i] = 180.0/pi * acos(vDot(cylDef, cylVec[i]) / ((motorVal[i]+0.001)/125))
        except ValueError:
            # TODO: This is a hack!
            cylAngle[i] = 0.0

    # =========================================================================
    # Draw the cylinders. For each cylinder, we:
    #   1. Set our color,
    #   2. Translate our POV to each motorBase,
    #   3. Rotate our view by the cylAngle calculated above, then
    #   4. Draw the cylinder.
    # We then reset our view.
    # =========================================================================
    for i in range(3):
        glColor4fv(motorColor[i])
        glTranslatef(motorBase[i][0], motorBase[i][1], motorBase[i][2])
        glRotatef(cylAngle[i], cylT[i][0], cylT[i][1], cylT[i][2]);

        # Rotate tail motor based on yaw motorVal[3]
        # TODO: rotation is messed up!
        #if i == 0:
        #    glRotatef(motorVal[3]-74, 0.0, 1.0, 0.0)

        #gluQuadricOrientation(quadratic, GLU_OUTSIDE);
        gluCylinder(quadratic, 0.3, 0.3, (motorVal[i]-0.001)/400, 32, 32);

        # Reset view.
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -3.0)
        glRotatef(-90.0, 1.0, 0.0, 0.0)


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
    global quadratic

    quadratic = quadratic = gluNewQuadric()

    glClearColor(0.0, 0.0, 0.0, 0.0)   # Clear background color to black.
    glClearDepth(1.0)                  # Enable clearing of the depth buffer.
    glDepthFunc(GL_LESS)               # Type of depth test.
    glEnable(GL_DEPTH_TEST)            # Enable depth testing.
    glShadeModel(GL_SMOOTH)            # Enable smooth color shading.

    glMatrixMode(GL_PROJECTION)   # Specify which matrix is the current matrix.
    glLoadIdentity()              # Reset the projection matrix.

    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)


# =============================================================================
# Threads
# =============================================================================
class TelemetrySubscriber(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running and not rospy.is_shutdown():
            rospy.Subscriber("tricopter_telemetry", Telemetry, telCallback, queue_size=1)
            rospy.spin()

class Visualizer(threading.Thread):
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


###############################################################################

if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("tricopter_visualization")

    try:
        tel = TelemetrySubscriber()
        tel.start()
        vis = Visualizer()
        vis.start()
        raw_input("Hit <enter> to quit.")

        # Stop the loops.
        tel.running = False
        vis.running = False

        # Wait for threads to finish jobs.
        tel.join()
        vis.join()

    except rospy.ROSInterruptException:
        pass

