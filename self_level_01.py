#       self_level_(series).py is a framework to provide a closed loop
#       controller to the surfaces on the aircraft by reading the stabilized gyro
#	angles from the BerryIMU.
#	This is the base code needed to get usable angles from a BerryIMU 
#	using a Complementary filter. The readings can be improved by 
#	adding more filters, E.g Kalman, Low pass, median filter, etc..
#       See berryIMU.py for more advanced code.
#
#	For this code to work correctly, BerryIMU must be facing the
#	correct way up. This is when the Skull Logo on the PCB is facing down.
#
#       Both the BerryIMUv1 and BerryIMUv2 are supported
#
#	http://ozzmaker.com/

from __future__ import division
import time
import math
import IMU.IMU as IMU
import datetime
import os
import Adafruit_PCA9685
import bmp280driver

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
throttle_servo_min = 275    # Min pulse length out of 4096
throttle_servo_max = 450    # Max pulse length out of 4096
rudder_servo_min = 275
rudder_servo_max = 450
aileron_servo_min = 275
aileron_servo_max = 450
elevator_servo_min = 275
elevator_servo_max = 450

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

# Moving average filter settings for Roll, Pitch, Yaw readings

combined_filter_length = 40
CFangleX_filter_length = combined_filter_length
CFangleY_filter_length = combined_filter_length
CFangleZ_filter_length = combined_filter_length
tiltHeading_filter_length = combined_filter_length

CFangleX_filtered = 0.0
CFangleX_reading = [0.0] * CFangleX_filter_length
CFangleY_filtered = 0.0
CFangleY_reading = [0.0] * CFangleY_filter_length
CFangleZ_filtered = 0.0
CFangleZ_reading = [0.0] * CFangleZ_filter_length
tiltHeading_filtered = 0.0
tiltHeading_reading = [0.0]*tiltHeading_filter_length

# Specify commanded plane attitude

heading_command = 0.0
yaw_command = 0.0
roll_command = 0.0
pitch_command = 0.0
throttle_command = 0.0

heading_error = 0.0
yaw_error = 0.0
roll_error = 0.0
pitch_error = 0.0
velocity_error = 0.0

heading = 0.0
yaw = 0.0
roll = 0.0
pitch = 0.0
velocity = 0.0

heading_p_gain = 0
yaw_p_gain = 0.0
roll_p_gain = 3.0
pitch_p_gain = 3.0
velocity_p_gain = 3.0

heading_i_gain = 0.0
yaw_i_gain = 0.0
roll_i_gain = 0.0
pitch_i_gain = 0.0
velocity_i_gain = 0.0

heading_d_gain = 0.0
yaw_d_gain = 0.0
roll_d_gain = 0.0
pitch_d_gain = 0.0
velocity_d_gain = 0.0

rudder_mid_pos = (rudder_servo_max + rudder_servo_min)/2
aileron_mid_pos = (aileron_servo_max + aileron_servo_min)/2
elevator_mid_pos = (elevator_servo_max + elevator_servo_min)/2

rudder_command = 0
aileron_command = 0
elevator_command = 0
throttle_command = 0

IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)


a = datetime.datetime.now()
pwm.set_pwm(0, 0, throttle_servo_min)
time.sleep(1)

# *******************************ROLL AND PITCH CALIBRATION START******************************************

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
print("X_offset = %5.2f" % (X_offset))
print("Y_offset = %5.2f" % (Y_offset))
print("Z_offset = %5.2f" % (Z_offset))
print("Head_offset = %5.2f" % (Head_offset))
time.sleep(5)

# ***CALIBRATION END****SELF-LEVEL LOOP START*****************************

while True:	
    #Read the accelerometer,gyroscope and magnetometer values
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
    #print("Loop Time | ", LP, " |")


    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN
    rate_gyr_z =  GYRz * G_GAIN


    #Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP
    gyroZangle+=rate_gyr_z*LP
    #print("gyroXangle: ", gyroXangle)
    #print("gyroYangle: ", gyroYangle)
    #print("gyroZangle: ", gyroZangle)


    #Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
    AccZangle =  (math.atan2(ACCx,ACCy)+M_PI)*RAD_TO_DEG

    #convert the values to -180 and +180
    AccXangle -= 180.0
    AccZangle -= 180.0
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0

    #Complementary filter used to combine the accelerometer and gyro values.
    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
    CFangleZ=AA*(CFangleZ+rate_gyr_z*LP) +(1 - AA) * AccZangle

    CFangleX_reading.append(CFangleX)
    del CFangleX_reading[0]
    CFangleX_filtered = sum(CFangleX_reading)/len(CFangleX_reading)

    CFangleY_reading.append(CFangleY)
    del CFangleY_reading[0]
    CFangleY_filtered = sum(CFangleY_reading)/max(len(CFangleY_reading),1)

    CFangleZ_reading.append(CFangleZ)
    del CFangleZ_reading[0]
    CFangleZ_filtered = sum(CFangleZ_reading)/max(len(CFangleZ_reading),1)

    #Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI

    #Only have our heading between 0 and 360
    #if heading < 0:
        #heading += 360

    #Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accZnorm = ACCz/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

    #Calculate pitch, roll and yaw
    pitch = math.asin(accXnorm)
    if pitch > 89:
        pitch = 89
    if pitch < -89:
        pitch = -89
    roll_intermediate = accYnorm/math.cos(pitch)
    if abs(roll_intermediate) > 1.0:
        roll_intermediate = 1.0
    roll = -math.asin(roll_intermediate)
    yaw = math.asin(accZnorm)

    #Calculate the new tilt compensated values
    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
    magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

    #Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    tiltHeading_reading.append(tiltCompensatedHeading)
    del tiltHeading_reading[0]
    tiltHeading_filtered = sum(tiltHeading_reading)/max(len(tiltHeading_reading),1)

    #Calculate altitude
    #altitude = 10^(log(10)*(pressure

    #if tiltCompensatedHeading < 0:
      #  tiltCompensatedHeading += 360

    if 0:			#Change to '0' to stop showing pitch, roll, yaw, heading and tilt compensated heading
        print ("\033[1;34;40m Pitch %5.2f Roll %5.2f Yaw %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (pitch*180/M_PI, roll*180/M_PI, yaw*180/M_PI, heading, tiltCompensatedHeading)),

    if 0:			#Change to '0' to stop showing the raw values from the Compass
        print ("\033[1;34;40m MAGx %5.2f MAGy %5.2f MAGz %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (MAGx, MAGy, MAGz, heading, tiltCompensatedHeading)),

    if 0:			#Change to '0' to stop showing the angles from the accelerometer
        print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f  \033[0m  " % (AccXangle, AccYangle)),

    if 0:			#Change to '0' to stop  showing the angles from the gyro
        print ("\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f" % (gyroXangle,gyroYangle,gyroZangle)),

    if 0:			#Change to '0' to stop  showing the angles from the complementary filter
        print ("\033[1;35;40m   \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m" % (CFangleX_filtered,CFangleY_filtered)),
        
    if 0:			#Change to '0' to stop  showing the heading
        print ("HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading))
        

    heading_error = heading_command - tiltHeading_filtered
    rudder_pos = int(rudder_mid_pos - heading_p_gain*heading_error)
    if (rudder_pos < rudder_servo_min):
        rudder_pos = rudder_servo_min
    if (rudder_pos > rudder_servo_max):
        rudder_pos = rudder_servo_max

##    yaw_error = yaw_command + CFangleZ_filtered + Z_offset
##    rudder_pos = int(rudder_mid_pos - yaw_p_gain*yaw_error)  
##    if (rudder_pos < rudder_servo_min):
##        rudder_pos = rudder_servo_min
##    if (rudder_pos > rudder_servo_max):
##        rudder_pos = rudder_servo_max

    roll_error = roll_command - CFangleX_filtered + X_offset
    aileron_pos = int(aileron_mid_pos + roll_p_gain*roll_error)
    if (aileron_pos < aileron_servo_min):
        aileron_pos = aileron_servo_min
    if (aileron_pos > aileron_servo_max):
        aileron_pos = aileron_servo_max

    pitch_error = pitch_command - CFangleY_filtered + Y_offset
    elevator_pos = int(elevator_mid_pos - pitch_p_gain*pitch_error)
    if (elevator_pos < elevator_servo_min):
      elevator_pos = elevator_servo_min
    if (elevator_pos > elevator_servo_max):
      elevator_pos = elevator_servo_max


    #pwm.set_pwm(0, 0, throttle_servo_min+20)
    rudder_pos = int(rudder_mid_pos)
    throttle_pos = 250

    pwm.set_pwm(0, 0, throttle_pos)

    pwm.set_pwm(1, 0, aileron_pos)

    pwm.set_pwm(2, 0, elevator_pos)

    pwm.set_pwm(3, 0, rudder_pos)

    print("throttle %d rudder %d elevator %d aileron %d" % (throttle_pos, rudder_pos, elevator_pos, aileron_pos))

    #slow program down a bit, makes the output more readable
    #time.sleep(0.03)


