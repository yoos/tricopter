#int32 time

# DCM values. ROS message arrays are currently limited to a maximum of only one
# dimension.
float32[3] dcmX
float32[3] dcmY
float32[3] dcmZ

# Rotation data
float32[3] rotationVector

# Motor values
float32[4] motorVals

# PID gains for X, Y, Z axes.
float32[3] pidGains

