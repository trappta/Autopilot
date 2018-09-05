import Adafruit_PCA9685

# ----- Constants -----

# Configure min and max servo pulse lengths
throttle_servo_off = 275
throttle_servo_min = 285
throttle_servo_max = 450
rudder_servo_min   = 250     # Min pulse length out of 4096
rudder_servo_max   = 425     # Max pulse length out of 4096
aileron_servo_min  = 275
aileron_servo_max  = 450
elevator_servo_min = 450
elevator_servo_max = 275

rudder_mid_pos = int((rudder_servo_max + rudder_servo_min)/2)
aileron_mid_pos = int((aileron_servo_max + aileron_servo_min)/2)
elevator_mid_pos = int((elevator_servo_max + elevator_servo_min)/2)

class Servo:

    def __init__(self, channel, servo_min, servo_max, is_reversed):

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        # Set frequency to 60hz, good for servos.
        self.pwm.set_pwm_freq(60)

        self.servo_min = servo_min
        self.servo_max = servo_max
        self.is_reversed = is_reversed

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_position(self, percent):

        if (percent > 100)
            percent = 100
        if (percent < 0)
            percent = 0

        if (self.is_reversed)
            pos = int((self.servo_max - self.servo_min)*((100 - float(percent)) / 100.0) + self.servo_min)
        else
            pos = int((self.servo_max - self.servo_min)*(float(percent) / 100.0) + self.servo_min)

        self.pwm.set_pwm(self.channel, 0, pos)
