import pygame
import sys
import time
import string
import serial

BUAD_RATE = 57600
SERIAL_PORT = 3

TX_FREQUENCY = 50.0 #Hz
TX_PERIOD = 1/TX_FREQUENCY

NUM_AXES = 5
NUM_BUTTONS = 10

THROTTLE_AXIS = 1
PITCH_AXIS = 3
ROLL_AXIS = 4
YAW_AXIS = 2

THROTTLE_SCALE = 999
PITCH_SCALE = 999
ROLL_SCALE = 999
YAW_SCALE = 999

AXIS_POS_DEADZONE = [0.00, 0.13, 0.03, 0.09, 0.16, 0.03] #still 6 numbers long to accomidate deadzone test
AXIS_NEG_DEADZONE = [-0.00, -0.00 -0.03, -0.09, -0.09, -0.03]

axisValue = [0.0 for x in range(NUM_AXES)]
buttonValue = 0

throttle = 0;
pitch = 0.0
roll = 0.0
yaw = 0.0

THROTTLE_TYPE = 0
d_throttle = 0.0
throttle = 0
THROTTLE_SCALE = 499
D_THROTTLE_SCALE = 2
D_THROTTLE_PWR = 3
THROTTLE_MIN = 0
THROTTLE_MAX = 999





# Initialize Joystick
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

# Initialize Serial Port
ser = serial.Serial(SERIAL_PORT, BUAD_RATE)

count = 0

telemetry_str = ""
t_last = time.clock()


while 1:
    # Collect telemetry
    if (ser.inWaiting() >= 1):
        while (ser.inWaiting() > 0):
            byte = ser.read(1)
            telemetry_str += byte
            if (byte == '\r'):
                # tlm_file.writelines(telemetry_str)
                telemetry = [int(i) for i in string.split(telemetry_str)]
                print telemetry
                telemetry_str = ""

    #Every period, grab latest joystick events and send to copter
    if (time.clock() - TX_PERIOD >= t_last):
        t_change = time.clock() - t_last
        t_last = time.clock()

        #Store events
        for event in pygame.event.get():
            if (event.type == pygame.QUIT): sys.exit()
            if (event.type == pygame.JOYAXISMOTION): axisValue[event.axis] = event.value
            if (event.type == pygame.JOYBUTTONDOWN): 
                buttonValue += pow(2, event.button)
                print buttonValue
            if (event.type == pygame.JOYBUTTONUP): buttonValue -= pow(2, event.button)

            #Ignore joystick movement within the deadzones
            axisValue = [0.0 if j < i < k else i for i, j, k in zip(axisValue, AXIS_NEG_DEADZONE, AXIS_POS_DEADZONE)]
            #for i in range(NUM_AXES):
                #if (AXIS_NEG_DEADZONE[i] < axisValue[i] < AXIS_POS_DEADZONE[i]):
                    #axisValue[i] = 0.0

        if (1):
            if (axisValue[1] < 0.0): d_throttle += abs(pow(-axisValue[1] * D_THROTTLE_SCALE, D_THROTTLE_PWR))
            else: d_throttle -= abs(pow(-axisValue[1] * D_THROTTLE_SCALE, D_THROTTLE_PWR))
            #print -axisValue[1], d_throttle
            if (abs(d_throttle) >= 1.0):
                throttle += int(d_throttle)
                throttle = min(throttle, THROTTLE_MAX)
                throttle = max(THROTTLE_MIN, throttle)
                d_throttle -= int(d_throttle)
        else:
            throttle = 499 + int(-axisValue[1] * THROTTLE_SCALE)

        pitch = int(-axisValue[PITCH_AXIS] * PITCH_SCALE)
        roll = int(axisValue[ROLL_AXIS] * ROLL_SCALE)
        yaw = int(-axisValue[YAW_AXIS] * YAW_SCALE)

        count += 1
        checksum = throttle + yaw + pitch + roll + buttonValue

        command = list()
        command.append(str(count).zfill(5))
        command.append(str(throttle).zfill(3))
        command.append(str(yaw).zfill(4))
        command.append(str(pitch).zfill(4))
        command.append(str(roll).zfill(4))
        command.append(str(buttonValue).zfill(4))
        command.append(str(checksum).zfill(5))

        command_ascii = string.join(command, ' ') + '\r'
        #print command_ascii
        ser.write(command_ascii)
