# Test program for autoleveling

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
pitch_command = 20.0
yaw_command = 0.0
throttle_command = 0
print("start")
i = 0
while i < 200000:
    print(i)
    i = i+1
    ac.attitude_control(heading_command,roll_command,pitch_command,yaw_command,throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure)