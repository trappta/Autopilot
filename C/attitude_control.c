#include "Adafruit-PWM-Servo-Driver-Library/Adafruit_PWMServoDriver.h"
//import bmp280driver

// ----- Constants -----

// Configure min and max servo pulse lengths
int throttle_servo_off = 275;
int throttle_servo_min = 285;
int throttle_servo_max = 450;
int rudder_servo_min   = 275;     // Min pulse length out of 4096
int rudder_servo_max   = 450;     // Max pulse length out of 4096
int aileron_servo_min  = 275;
int aileron_servo_max  = 450;
int elevator_servo_min = 275;
int elevator_servo_max = 450;

// Proportional control gains
double p_gain = 4.0;

int IMU_upside_down = 0;     // Change calculations depending on IMu orientation. 
                        // 0 = Correct side up. This is when the skull logo is facing down
                        // 1 = Upside down. This is when the skull logo is facing up

double RAD_TO_DEG = 57.29578;
double M_PI = 3.14159265358979323846;
double G_GAIN = 0.070;  // [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
double AA =  0.40;      // Complementary filter constant
double MAG_LPF_FACTOR = 0.4; 	// Low pass filter constant magnetometer
double ACC_LPF_FACTOR = 0.4; 	// Low pass filter constant for accelerometer
int ACC_MEDIANTABLESIZE = 30;    	// Median filter table size for accelerometer. Higher = smoother but a longer delay
int MAG_MEDIANTABLESIZE = 30;    	// Median filter table size for magnetometer. Higher = smoother but a longer delay

// More Kalman filter constants
double Q_angle = 0.02;
double Q_gyro = 0.0015;
double R_angle = 0.005;

Adafruit_PWMServoDriver pwm = NULL;

double gyroXangle = 0.0;
double gyroYangle = 0.0;
double gyroZangle = 0.0;
double CFangleX = 0.0;
double CFangleY = 0.0;
double CFangleZ = 0.0; // Uneeded in new?
double CFangleXFiltered = 0.0;
double CFangleYFiltered = 0.0;
double kalmanX = 0.0;
double kalmanY = 0.0;
double oldXMagRawValue = 0;
double oldYMagRawValue = 0;
double oldZMagRawValue = 0;
double oldXAccRawValue = 0;
double oldYAccRawValue = 0;
double oldZAccRawValue = 0;

// Setup the tables for the median filter. Fill them all with '1' so we don't get a divide by zero error
double acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE;
double acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE;
double acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE;
double acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE;
double acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE;
double acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE;
double mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE;
double mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE;
double mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE;
double mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE;
double mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE;
double mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE;

// Kalman filter variables
double y_bias = 0.0;
double x_bias = 0.0;
double XP_00 = 0.0;
double XP_01 = 0.0;
double XP_10 = 0.0;
double XP_11 = 0.0;
double YP_00 = 0.0;
double YP_01 = 0.0;
double YP_10 = 0.0;
double YP_11 = 0.0;
double KFangleX = 0.0;
double KFangleY = 0.0;

double pressure = 0.0;
double altitude = 0.0;

// Moving average filter settings for Roll, Pitch, Yaw readings

double combined_filter_length = 100;
double CFangleX_filter_length = combined_filter_length;
double CFangleY_filter_length = combined_filter_length;
double CFangleZ_filter_length = combined_filter_length;
double tiltHeading_filter_length = combined_filter_length;

double CFangleX_filtered = 0.0;
double CFangleX_reading = [0.0] * CFangleX_filter_length;
double CFangleY_filtered = 0.0;
double CFangleY_reading = [0.0] * CFangleY_filter_length;
double CFangleZ_filtered = 0.0;
double CFangleZ_reading = [0.0] * CFangleZ_filter_length;
double tiltHeading_filtered = 0.0;
double tiltHeading_reading = [0.0] * tiltHeading_filter_length;

//double time_a = datetime.datetime.now(); //FIXME

void init() {
    // Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PWMServoDriver();

    // Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(60);
}

// Helper function to make setting a servo pulse width simpler.
void set_servo_pulse(int channel, int pulse) {
    int pulse_length = 1000000;    // 1,000,000 us per second
    pulse_length = floor(pulse_length / 60);       // 60 Hz
    //printf('{0}us per period'.format(pulse_length))
    pulse_length = floor(pulse_length / 4096);     // 12 bits of resolution
    //printf('{0}us per bit'.format(pulse_length))
    pulse *= 1000;
    pulse = floor(pulse / pulse_length);
    pwm.set_pwm(channel, 0, pulse);
}

double kalmanFilterY (self, accAngle, gyroRate, DT) {
    double y=0.0;
    double S=0.0;

    KFangleY = KFangleY + DT * (gyroRate - y_bias);

    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT );
    YP_01 = YP_01 + ( - DT * YP_11 );
    YP_10 = YP_10 + ( - DT * YP_11 );
    YP_11 = YP_11 + ( + Q_gyro * DT );

    double y = accAngle - KFangleY;
    double S = YP_00 + R_angle;
    double K_0 = YP_00 / S;
    double K_1 = YP_10 / S;
    
    KFangleY = KFangleY + ( K_0 * y );
    y_bias = y_bias + ( K_1 * y );
    
    YP_00 = YP_00 - ( K_0 * YP_00 );
    YP_01 = YP_01 - ( K_0 * YP_01 );
    YP_10 = YP_10 - ( K_1 * YP_00 );
    YP_11 = YP_11 - ( K_1 * YP_01 );
    
    return KFangleY;
}

double kalmanFilterX (self, accAngle, gyroRate, DT) {
    double x = 0.0;
    double S = 0.0;

    KFangleX = KFangleX + DT * (gyroRate - x_bias);

    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT );
    XP_01 = XP_01 + ( - DT * XP_11 );
    XP_10 = XP_10 + ( - DT * XP_11 );
    XP_11 = XP_11 + ( + Q_gyro * DT );

    double x = accAngle - double KFangleX;
    double S = double XP_00 + R_angle;
    double K_0 = double XP_00 / S;
    double K_1 = double XP_10 / S;
    
    KFangleX = KFangleX + ( K_0 * x );
    x_bias = x_bias + ( K_1 * x );
    
    XP_00 = XP_00 - ( K_0 * XP_00 );
    XP_01 = XP_01 - ( K_0 * XP_01 );
    XP_10 = XP_10 - ( K_1 * XP_00 );
    XP_11 = XP_11 - ( K_1 * XP_01 );
    
    return KFangleX;
}

void attitude_control(heading_command, roll_command, pitch_command, yaw_command, throttle_command, X_offset, Y_offset, Z_offset, Head_offset, pressure) {

    double heading_p_gain = p_gain;
    double yaw_p_gain = p_gain;
    double roll_p_gain = p_gain;
    double pitch_p_gain = p_gain;

    int rudder_mid_pos = (rudder_servo_max + rudder_servo_min)/2;
    int aileron_mid_pos = (aileron_servo_max + aileron_servo_min)/2;
    int elevator_mid_pos = (elevator_servo_max + elevator_servo_min)/2;

    //Read the accelerometer, gyroscope and magnetometer values
    double ACCx = IMU.readACCx();
    double ACCy = IMU.readACCy();
    double ACCz = IMU.readACCz();
    double GYRx = IMU.readGYRx();
    double GYRy = IMU.readGYRy();
    double GYRz = IMU.readGYRz();
    double MAGx = IMU.readMAGx();
    double MAGy = IMU.readMAGy();
    double MAGz = IMU.readMAGz();

    // Calculate loop Period(dt). How long between Gyro Reads
    double time_b = datetime.datetime.now() - time_a;
    time_a = datetime.datetime.now();
    double dt = time_b.microseconds/(1000000*1.0);
    //printf("Loop Time | ", dt, " |");

    // Read altimeter value
    double pressure = bmp280driver.readALT();


    ////////////////////////////////////////////////////////////////////////////////////////////// 
    //////// Apply low pass filter ////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    double MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
    double MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
    double MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
    double ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
    double ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
    double ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);

    oldXMagRawValue = MAGx;
    oldYMagRawValue = MAGy;
    oldZMagRawValue = MAGz;
    oldXAccRawValue = ACCx;
    oldYAccRawValue = ACCy;
    oldZAccRawValue = ACCz;

    ////////////////////////////////////////////////////////////////////////////////// 
    //////// Median filter for accelerometer ////////
    //////////////////////////////////////////////////////////////////////////////////
    // cycle the table
    for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ) {
        acc_medianTable1X[x] = acc_medianTable1X[x-1];
        acc_medianTable1Y[x] = acc_medianTable1Y[x-1];
        acc_medianTable1Z[x] = acc_medianTable1Z[x-1];
    }

    // Insert the lates values
    acc_medianTable1X[0] = ACCx;
    acc_medianTable1Y[0] = ACCy;
    acc_medianTable1Z[0] = ACCz;

    // Copy the tables
    acc_medianTable2X = acc_medianTable1X[:];
    acc_medianTable2Y = acc_medianTable1Y[:];
    acc_medianTable2Z = acc_medianTable1Z[:];

    // Sort table 2
    acc_medianTable2X.sort();
    acc_medianTable2Y.sort();
    acc_medianTable2Z.sort();

    // The middle value is the value we are interested in
    ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
    ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
    ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];



    ////////////////////////////////////////////////////////////////////////////////// 
    //////// Median filter for magnetometer ////////
    //////////////////////////////////////////////////////////////////////////////////
    // cycle the table
    for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ) {
        mag_medianTable1X[x] = mag_medianTable1X[x-1];
        mag_medianTable1Y[x] = mag_medianTable1Y[x-1];
        mag_medianTable1Z[x] = mag_medianTable1Z[x-1];
    }

    // Insert the latest values    
    mag_medianTable1X[0] = MAGx;
    mag_medianTable1Y[0] = MAGy;
    mag_medianTable1Z[0] = MAGz;

    // Copy the tables
    mag_medianTable2X = mag_medianTable1X[:];
    mag_medianTable2Y = mag_medianTable1Y[:];
    mag_medianTable2Z = mag_medianTable1Z[:];

    // Sort table 2
    mag_medianTable2X.sort();
    mag_medianTable2Y.sort();
    mag_medianTable2Z.sort();

    // The middle value is the value we are interested in
    MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
    MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
    MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];



    //Convert Gyro raw to degrees per second
    double rate_gyr_x = GYRx * G_GAIN;
    double rate_gyr_y = GYRy * G_GAIN;
    double rate_gyr_z = GYRz * G_GAIN;

    //Calculate the angles from the gyro.
    gyroXangle += rate_gyr_x*dt;
    gyroYangle += rate_gyr_y*dt;
    gyroZangle += rate_gyr_z*dt;
    //printf("double gyroXangle: ", double gyroXangle)
    //printf("double gyroYangle: ", double gyroYangle)
    //printf("double gyroZangle: ", double gyroZangle)


    //Convert Accelerometer values to degrees
    AccXangle = (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG;
    AccYangle = (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG;
    AccZangle = (math.atan2(ACCx,ACCy)+M_PI)*RAD_TO_DEG;




    // Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up
    // Two different pieces of code are used depending on how your IMU is mounted
    if (IMU_upside_down) {         // If IMU is upside down E.g Skull logo is facing up.
        if (AccXangle > 180) {
            AccXangle -= 360.0;
            AccYangle -= 90;
        }
        if (AccYangle > 180) {
            AccYangle -= 360.0;
        }
    }
    else {                         // If IMU is up the correct way E.g Skull logo is facing down.
        AccXangle -= 180.0;
        if (AccYangle > 90)
            AccYangle -= 270.0;
        else
            AccYangle += 90.0;
    }

    // Convert the values to -180 and +180
    //AccXangle -= 180.0
    //AccZangle -= 180.0
    //if AccYangle > 90:
    //    AccYangle -= 270.0
    //else:
    //    AccYangle += 90.0

    // Complementary filter used to combine the accelerometer and gyro values.
    CFangleX = AA*(CFangleX+rate_gyr_x*dt) +(1 - AA) * AccXangle;
    CFangleY = AA*(CFangleY+rate_gyr_y*dt) +(1 - AA) * AccYangle;
    CFangleZ = AA*(CFangleZ+rate_gyr_z*dt) +(1 - AA) * AccZangle;

    // Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,dt);
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,dt);


    //double CFangleX_reading.append(double CFangleX)
    //del double CFangleX_reading[0]
    //double CFangleX_filtered = sum(double CFangleX_reading)/max(len(double CFangleX_reading),1)

    //double CFangleY_reading.append(double CFangleY)
    //del double CFangleY_reading[0]
    //double CFangleY_filtered = sum(double CFangleY_reading)/max(len(double CFangleY_reading),1)

    CFangleZ_reading.append(CFangleZ);
    del CFangleZ_reading[0];
    CFangleZ_filtered = sum(CFangleZ_reading)/max(len(CFangleZ_reading),1);



    if (IMU_upside_down)
        MAGy = -MAGy;

    // Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI;

    // Only have our heading between 0 and 360
    if (heading < 0)
        heading += 360;

    //Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz);
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz);
    accZnorm = ACCz/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz);

    //Calculate pitch, roll and yaw
    pitch = math.asin(accXnorm);
    if (pitch > 89)
        pitch = 89;
    if (pitch < -89)
        pitch = -89;
    roll_intermediate = accYnorm/math.cos(pitch);
    if (abs(roll_intermediate) > 1.0)
        roll_intermediate = 1.0;
    roll = -math.asin(roll_intermediate);
    yaw = math.asin(accZnorm);

    //Calculate the new tilt compensated values
    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch);
    magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch);

    //Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI;

    //Only have our heading between 0 and 360
    if (tiltCompensatedHeading < 0)
        tiltCompensatedHeading += 360;

    //double tiltHeading_reading.append(tiltCompensatedHeading)
    //del double tiltHeading_reading[0]
    //tiltHeading_filtered = sum(double tiltHeading_reading)/max(len(double tiltHeading_reading),1)

    //Calculate altitude
    //altitude = 10^(log(10)*(pressure


    if (0)	// Change to '0' to stop showing pitch, roll, yaw, heading and tilt compensated heading
        printf("\033[1;34;40m Pitch %5.2f Roll %5.2f Yaw %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (pitch*180/M_PI, roll*180/M_PI, yaw*180/M_PI, heading, tiltCompensatedHeading));

    if (0)	// Change to '0' to stop showing the raw values from the Compass
        printf("\033[1;34;40m MAGx %5.2f MAGy %5.2f MAGz %5.2f HEADING %5.2f TCH %5.2f \033[0m \n " % (MAGx, MAGy, MAGz, heading, tiltCompensatedHeading));

    if (0)	// Change to '0' to stop showing the angles from the accelerometer
        printf("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f  \033[0m  " % (AccXangle, AccYangle)),

    if (0)	// Change to '0' to stop  showing the angles from the gyro
        printf("\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f" % (double gyroXangle,double gyroYangle,double gyroZangle));

    if (0)	// Change to '0' to stop  showing the angles from the complementary filter
        printf("\033[1;35;40m   \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m" % (double CFangleX_filtered,double CFangleY_filtered));

    if (0)	// Change to '0' to stop  showing the heading
        printf("HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading));


    //heading_error = heading_command - tiltHeading_filtered
    heading_error = heading_command - tiltCompensatedHeading;
    rudder_pos = int(rudder_mid_pos - heading_p_gain*heading_error);
    if (rudder_pos < rudder_servo_min)
        rudder_pos = rudder_servo_min;
    if (rudder_pos > rudder_servo_max)
        rudder_pos = rudder_servo_max;


    //yaw_error = yaw_command + CFangleZ_filtered + Z_offset
    yaw_error = yaw_command + CFangleZ_filtered + Z_offset;
    rudder_pos = int(rudder_mid_pos - yaw_p_gain*yaw_error);
    if (rudder_pos < rudder_servo_min)
        rudder_pos = rudder_servo_min;
    if (rudder_pos > rudder_servo_max)
        rudder_pos = rudder_servo_max;

    //roll_error = roll_command - CFangleX_filtered + X_offset
    roll_error = roll_command - kalmanX + X_offset;
    aileron_pos = int(aileron_mid_pos + roll_p_gain*roll_error);
    if (aileron_pos < aileron_servo_min)
        aileron_pos = aileron_servo_min;
    if (aileron_pos > aileron_servo_max)
        aileron_pos = aileron_servo_max;

    //pitch_error = pitch_command - CFangleY_filtered + Y_offset;
    pitch_error = pitch_command - kalmanY + Y_offset;
    elevator_pos = int(elevator_mid_pos - pitch_p_gain*pitch_error);
    if (elevator_pos < elevator_servo_min)
      elevator_pos = elevator_servo_min;
    if (elevator_pos > elevator_servo_max)
      elevator_pos = elevator_servo_max;

    //rudder_pos = int(rudder_mid_pos);
    throttle_pos = int((throttle_servo_max-throttle_servo_min)*(float(throttle_command) / 100.0) + throttle_servo_min);
    pwm.set_pwm(0, 0, throttle_pos);
    pwm.set_pwm(1, 0, aileron_pos);
    pwm.set_pwm(2, 0, elevator_pos);
    //double pwm.set_pwm(3, 0, rudder_pos);
    pwm.set_pwm(3, 0, 375);  // steer straight for testing the takeoff.py profile

    //printf("throttle %d rudder %d elevator %d aileron %d" % (throttle_pos, rudder_pos, elevator_pos, aileron_pos));

    // Slow program down a bit, makes the output more readable
    //time.sleep(0.03);
}
