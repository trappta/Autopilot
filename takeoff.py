# Experimental takeoff program

from __future__ import division
import calibrate_sensors
import time
import math
import IMU.IMU as IMU
import datetime
import os
import Adafruit_PCA9685
import bmp280driver
from attitude_control import attitude_control

ac = attitude_control()

X = calibrate_sensors.calibrate()

print("Calibration Complete")
print("X_offset = %5.2f" % (X[0]))
print("Y_offset = %5.2f" % (X[1]))
print("Z_offset = %5.2f" % (X[2]))
print("Head_offset = %5.2f" % (X[3]))
print("pressure = %5.2f" % (X[4]))

X_offset = X[0]
Y_offset = X[1]
Z_offset = X[2]
Head_offset = X[3]
pressure = X[4]

#time.sleep(10)

# start on runway at 0% throttle
heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 0

print("start on runway at 0% throttle")
for i in range(500):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

# start down runway at 50% throttle
heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 50
#print("start down runway at 50% throttle")
#for i in range(1000):
#    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

# increase to 100% throttle and pitch up
heading_command = 0.0
roll_command = 0.0
pitch_command = -20.0
yaw_command = 0.0
throttle_command = 100
print("increase to 100% throttle and pitch up 10 degrees")
for i in range(2000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

# reduce throttle to 75% and continue ascent
heading_command = 0.0
roll_command = 0.0
pitch_command = 10.0
yaw_command = 0.0
throttle_command = 75
print("reduce throttle to 75% and continue ascent")
for i in range(1000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

# reduce throttle to 50% and level off
heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 50
print("reduce throttle to 50% and level off")
for i in range(1000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

# stop the test
heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 0
print("stop the test")
for i in range(1):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)
