"""
Author: Thomas Pardy
Date Modified: 2023-08-21
Ultrasonic sensor modules.
"""

import RPi.GPIO as GPIO
import time

class Ultrasonic():
    """
    Module for ultrasonic sensor
    """
    def __init__(self):
        self.trig_pin = 16
        self.echo_pin = 18
        self.time_out =  200 * 2 / 100 / 340 * 1e6# Max Distance*2 / 100 / 340 * 1e6

    def pulse(self, pin, toggle, time_out):
        """
        Return the length of the pulse (uS) or 0 if no pulse is returned before
        the timeout.
        """
        t0 = time.time()
        while (GPIO.input(pin) != toggle):
            if (time.time() - t0 > self.time_out*1e-6):
                return 0
        t0 = time.time()
        while (GPIO.input(pin) == toggle):
            if ((time.time() - t0 > self.time_out*1e-6)):
                return 0

        pulse_time = (time.time() - t0)*1e6

        return pulse_time


    def get_sonar(self):
        """
        Return the measured distance of pulse in cm
        """
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(1e-5) # 10us
        GPIO.output(self.trig_pin, GPIO.LOW)
        ping_time = self.pulse(self.echo_pin, GPIO.HIGH, self.time_out)
        print(ping_time)
        distance = ping_time * 340/ 2 / 1e4 # speed of sound 340m/s
        return distance


    def setup(self):

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trig_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)


    def loop(self):
        while True:
            distance = self.get_sonar()
            print("Distance: %.2f cm"%(distance))
            time.sleep(1)


if __name__ == '__main__':
    sensor = Ultrasonic()
    sensor.setup()

    try:
        sensor.loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
