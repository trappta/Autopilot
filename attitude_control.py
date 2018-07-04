import time
import math
import IMU.IMU as IMU
import datetime
import os
import Adafruit_PCA9685
import bmp280driver

# ----- Constants -----

# Configure min and max servo pulse lengths
throttle_servo_min = 275     # Min pulse length out of 4096
throttle_servo_max = 450    # Max pulse length out of 4096
rudder_servo_min = 275
rudder_servo_max = 450
aileron_servo_min = 275
aileron_servo_max = 450
elevator_servo_min = 275
elevator_servo_max = 450

rudder_mid_pos = (rudder_servo_max + rudder_servo_min)/2
aileron_mid_pos = (aileron_servo_max + aileron_servo_min)/2
elevator_mid_pos = (elevator_servo_max + elevator_servo_min)/2

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

class attitude_control(object):

    def __init__(self):

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        # Set frequency to 60hz, good for servos.
        self.pwm.set_pwm_freq(60)

        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.CFangleZ = 0.0
        self.pressure = 0.0
        self.altitude = 0.0

        # Moving average filter settings for Roll, Pitch, Yaw readings

        self.combined_filter_length = 40
        self.CFangleX_filter_length = self.combined_filter_length
        self.CFangleY_filter_length = self.combined_filter_length
        self.CFangleZ_filter_length = self.combined_filter_length
        self.tiltHeading_filter_length = self.combined_filter_length

        self.CFangleX_filtered = 0.0
        self.CFangleX_reading = [0.0] * self.CFangleX_filter_length
        self.CFangleY_filtered = 0.0
        self.CFangleY_reading = [0.0] * self.CFangleY_filter_length
        self.CFangleZ_filtered = 0.0
        self.CFangleZ_reading = [0.0] * self.CFangleZ_filter_length
        self.tiltHeading_filtered = 0.0
        self.tiltHeading_reading = [0.0] * self.tiltHeading_filter_length

        self.time_a = datetime.datetime.now()

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(self, channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        self.pwm.set_pwm(channel, 0, pulse)

    def attitude_control(self, heading_command, roll_command, pitch_command, yaw_command, throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure):

        heading_p_gain = 2.5
        yaw_p_gain = 5.0
        roll_p_gain = 5.0
        pitch_p_gain = 5.0

#looooooooooooooooooooooooooop

        #Calculate loop Period(dt). How long between Gyro Reads
        time_b = datetime.datetime.now() - self.time_a
        self.time_a = datetime.datetime.now()
        dt = time_b.microseconds/(1000000*1.0)
        #print("Loop Time | ", dt, " |")

        #Read the accelerometer, gyroscope and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
        pressure = bmp280driver.readALT()

        #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * G_GAIN
        rate_gyr_y =  GYRy * G_GAIN
        rate_gyr_z =  GYRz * G_GAIN


        #Calculate the angles from the gyro.
        self.gyroXangle+=rate_gyr_x*dt
        self.gyroYangle+=rate_gyr_y*dt
        self.gyroZangle+=rate_gyr_z*dt
        #print("self.gyroXangle: ", self.gyroXangle)
        #print("self.gyroYangle: ", self.gyroYangle)
        #print("self.gyroZangle: ", self.gyroZangle)


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
        self.CFangleX=AA*(self.CFangleX+rate_gyr_x*dt) +(1 - AA) * AccXangle
        self.CFangleY=AA*(self.CFangleY+rate_gyr_y*dt) +(1 - AA) * AccYangle
        self.CFangleZ=AA*(self.CFangleZ+rate_gyr_z*dt) +(1 - AA) * AccZangle

        self.CFangleX_reading.append(self.CFangleX)
        del self.CFangleX_reading[0]
        self.CFangleX_filtered = sum(self.CFangleX_reading)/len(self.CFangleX_reading)

        self.CFangleY_reading.append(self.CFangleY)
        del self.CFangleY_reading[0]
        self.CFangleY_filtered = sum(self.CFangleY_reading)/max(len(self.CFangleY_reading),1)

        self.CFangleZ_reading.append(self.CFangleZ)
        del self.CFangleZ_reading[0]
        self.CFangleZ_filtered = sum(self.CFangleZ_reading)/max(len(self.CFangleZ_reading),1)

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

        self.tiltHeading_reading.append(tiltCompensatedHeading)
        del self.tiltHeading_reading[0]
        tiltHeading_filtered = sum(self.tiltHeading_reading)/max(len(self.tiltHeading_reading),1)

        #Calculate altitude
        #altitude = 10^(log(10)*(pressure

        #if tiltCompensatedHeading < 0:
        #    tiltCompensatedHeading += 360

        if 0:			#Change to '0' to stop showing pitch, roll, yaw, heading and tilt compensated heading
            print ("\033[1;34;40m Pitch %5.2f Roll %5.2f Yaw %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (pitch*180/M_PI, roll*180/M_PI, yaw*180/M_PI, heading, tiltCompensatedHeading)),

        if 0:			#Change to '0' to stop showing the raw values from the Compass
            print ("\033[1;34;40m MAGx %5.2f MAGy %5.2f MAGz %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (MAGx, MAGy, MAGz, heading, tiltCompensatedHeading)),

        if 0:			#Change to '0' to stop showing the angles from the accelerometer
            print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f  \033[0m  " % (AccXangle, AccYangle)),

        if 0:			#Change to '0' to stop  showing the angles from the gyro
            print ("\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f" % (self.gyroXangle,self.gyroYangle,self.gyroZangle)),

        if 0:			#Change to '0' to stop  showing the angles from the complementary filter
            print ("\033[1;35;40m   \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m" % (self.CFangleX_filtered,self.CFangleY_filtered)),

        if 0:			#Change to '0' to stop  showing the heading
            print ("HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading))


        heading_error = heading_command - tiltHeading_filtered
        rudder_pos = int(rudder_mid_pos - heading_p_gain*heading_error)
        if (rudder_pos < rudder_servo_min):
            rudder_pos = rudder_servo_min
        if (rudder_pos > rudder_servo_max):
            rudder_pos = rudder_servo_max

        yaw_error = yaw_command + self.CFangleZ_filtered + Z_offset
        rudder_pos = int(rudder_mid_pos - yaw_p_gain*yaw_error)
        if (rudder_pos < rudder_servo_min):
            rudder_pos = rudder_servo_min
        if (rudder_pos > rudder_servo_max):
            rudder_pos = rudder_servo_max

        roll_error = roll_command - self.CFangleX_filtered + X_offset
        aileron_pos = int(aileron_mid_pos + roll_p_gain*roll_error)
        if (aileron_pos < aileron_servo_min):
            aileron_pos = aileron_servo_min
        if (aileron_pos > aileron_servo_max):
            aileron_pos = aileron_servo_max

        pitch_error = pitch_command - self.CFangleY_filtered + Y_offset
        elevator_pos = int(elevator_mid_pos - pitch_p_gain*pitch_error)
        if (elevator_pos < elevator_servo_min):
          elevator_pos = elevator_servo_min
        if (elevator_pos > elevator_servo_max):
          elevator_pos = elevator_servo_max


        #rudder_pos = int(rudder_mid_pos)
        throttle_pos = int((throttle_servo_max-throttle_servo_min)*(float(throttle_command) / 100.0) + throttle_servo_min)

        self.pwm.set_pwm(0, 0, throttle_pos)

        self.pwm.set_pwm(1, 0, aileron_pos)

        self.pwm.set_pwm(2, 0, elevator_pos)

        self.pwm.set_pwm(3, 0, rudder_pos)

        print("throttle %d rudder %d elevator %d aileron %d" % (throttle_pos, rudder_pos, elevator_pos, aileron_pos))

        #slow program down a bit, makes the output more readable
        #time.sleep(0.03)
