"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""

from .pwm import Pwm

PIN_NUMBER = 12
PULSOUT_075_MS = 375
PULSOUT_225_MS = 1125
PAUSE = 20.0e-3
SERVO_DEGREE_LIMITATION = 180


class Servo:
    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        # self.pwn = Pwm(PIN_NUMBER)
        self.pwm = Pwm(number)
        self.pwm.enable()
        self.frequency = int(1 / PAUSE)
        self.pwm.set_frequency(self.frequency)

    def go_to(self, angle):
        """Go to specified target angle.

        Args:
            angle (float): -90 - 90 degrees. 0 means facing forward. Negative numbers turn to the left.
        """
        if angle < -90.0:
            angle = -90.0
        elif angle > 90.0:
            angle = 90.0
        else:
            angle

        # percent = (angle + 90) / 180 * self.full_pulse / self.period
        # pulse_out_diff = PULSOUT_225_MS - PULSOUT_075_MS
        # percent = ((angle + 90) / SERVO_DEGREE_LIMITATION * pulse_out_diff + PULSOUT_075_MS) / 100.0
        # print("angle: " + str(angle) + " percent: " + str(percent))

        max_pwm = 2.25/20
        min_pwm = 0.75/20
        duty_cycle_percent = ((((angle + 90) / SERVO_DEGREE_LIMITATION) * (max_pwm - min_pwm) + min_pwm))* 100
        self.pwm.set_duty_cycle(percent)


