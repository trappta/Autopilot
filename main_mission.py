# Main Mission Program
# Step 1. Calibrate the Aircraft sensors. Perform once at beginning.
# Step 2. Define a sequence of events (safe startup routine, takeoff, reach cruising altitude,
#             fly waypoints, determine mission complete, land, safe shutdown)
# Step 2 calls attitude control.

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

heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 0
print("start")
for i in range(2000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

heading_command = 10.0
roll_command = 10.0
pitch_command = 10.0
yaw_command = 0.0
throttle_command = 50
print("step 1")
for i in range(2000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

heading_command = -10.0
roll_command = -10.0
pitch_command = -10.0
yaw_command = 0.0
throttle_command = 100
print("step 2")
for i in range(2000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)

heading_command = 0.0
roll_command = 0.0
pitch_command = 0.0
yaw_command = 0.0
throttle_command = 0
print("step 3")
for i in range(2000):
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)
