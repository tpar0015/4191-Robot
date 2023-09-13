# Module code for Electromagnet
import RPi.GPIO as GPIO
import time
import sys
from pins import *

class Electromagnet:
    """
    Class module to control the electromagnet
    Functions include: Toggling the electromagnet, increasing or decreasing the strength of the electromagnet
    """
    def __init__(self, gpio_pin=12, frequency=1000, duty_cycle=50):
        self.gpio_pin = gpio_pin #PINS["electromagnet"]
        self.frequency = frequency
        self.duty_cycle = duty_cycle

        GPIO.setmode(GPIO.BCM) # Use BCM numbering
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(gpio_pin, frequency)
    
    def set_cycle(self):
        self.pwm.ChangeDutyCycle(duty_cycle)
    
    def turn_on(self):
        self.pwm.start(self.duty_cycle)

    def clean_up(self):
        self.pwm.stop()
        GPIO.cleanup()
        
        
if __name__ == "__main__":
    gpio_pin = 12
    frequency = 1000  
    duty_cycle = 50 

    electromagnet = Electromagnet(gpio_pin, frequency, duty_cycle)

    try:
        while True:
            try:
                duty_cycle = float(input("Enter duty cycle (0 to 100): "))
                electromagnet.set_cycle(duty_cycle)
            except ValueError:
                pass
    except KeyboardInterrupt:
        pass
    electromagnet.clean_up()
