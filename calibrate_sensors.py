from __future__ import division
import time
import math
import IMU.IMU as IMU
import datetime
import os
import Adafruit_PCA9685
import bmp280driver

def calibrate():

    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
    AA =  0.40      # Complementary filter constant

    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    CFangleZ = 0.0
    pressure = 0.0
    altitude = 0.0
    
    IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
    IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
    a = datetime.datetime.now()
    
    cal_length = 300
    X_cal = [0.0]*cal_length
    Y_cal = [0.0]*cal_length
    Z_cal = [0.0]*cal_length
    Head_cal = [0.0]*cal_length

    #cal_ready = raw_input("Is the plane level and ready for IMU calibration (Y/N)?  ")
    #type(cal_ready)
    cal_ready = 'Y'
    if (cal_ready =='Y'):
        print('Entering Calibration Routine')
        for i in range(0,cal_length):

            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()
            #pressure = bmp280driver.readALT()

            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            #print "Loop Time | %5.2f|" % ( LP ),


            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN


            #Calculate the angles from the gyro. 
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP


            #Convert Accelerometer values to degrees
            AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
            AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
            AccZangle =  (math.atan2(ACCx,ACCy)+M_PI)*RAD_TO_DEG

            #convert the values to -180 and +180
            AccXangle -= 180.0
            AccZangle -=180.0
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0

            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
            CFangleZ=AA*(CFangleZ+rate_gyr_z*LP) +(1 - AA) * AccZangle
            # gyro filter array
            X_cal[i] = CFangleX
            Y_cal[i] = CFangleY
            Z_cal[i] = CFangleZ
        
            #Calculate heading
            heading = 180 * math.atan2(MAGy,MAGx)/M_PI
            # magnetometer filter array
            Head_cal[i] = heading        

    X_offset = sum(X_cal)/max(len(X_cal), 1)
    Y_offset = sum(Y_cal)/max(len(Y_cal), 1)
    Z_offset = sum(Z_cal)/max(len(Z_cal),1)

    Head_offset = sum(Head_cal)/max(len(Head_cal),1)
    heading_command= Head_offset

    print("Calibration Complete")
    print("X_offset =  %5.2f" % (X_offset))
    print("Y_offset =  %5.2f" % (Y_offset))
    print("Z_offset = %5.2f" % (Z_offset))
    print("Head_offset = %5.2f" % (Head_offset))

    return(X_offset, Y_offset, Z_offset, Head_offset, pressure)
