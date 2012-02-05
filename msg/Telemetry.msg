#int32 time

# DCM values. ROS message arrays are currently limited to a maximum of only one
# dimension.
#float32[3] dcmX
#float32[3] dcmY
#float32[3] dcmZ

# DCM values, separated.
float32 dcmXX
float32 dcmXY
float32 dcmXZ
float32 dcmYX
float32 dcmYY
float32 dcmYZ
float32 dcmZX
float32 dcmZY
float32 dcmZZ

# Rotation data
#float32[3] rotationVector

# Motor values
float32 motorT
float32 motorR
float32 motorL
float32 servoT

# PID gains for X, Y, Z axes.
float32 P
float32 I
float32 D

