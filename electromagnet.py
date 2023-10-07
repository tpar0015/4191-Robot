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
    def __init__(self, gpio_pin=12, frequency=100, duty_cycle=50):
        self.gpio_pin = gpio_pin #PINS["electromagnet"]
        self.frequency = frequency
        self.duty_cycle = duty_cycle

        GPIO.setmode(GPIO.BCM) # Use BCM numbering
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(gpio_pin, frequency)
    
    def turn_on(self):
        "Toggles on"
        GPIO.output(self.gpio_pin, GPIO.LOW)
        time.sleep(1)
    
    def turn_off(self):
        "Toggles off"
        GPIO.output(self.gpio_pin, GPIO.HIGH)   # HIGH as circuit is active low
        self.pwm.start(0)
        self.pwm.stop(0)

    def pwm_on(self):
        "Turn on PWM"
        GPIO.output(self.gpio_pin, GPIO.LOW)    # circuit is active low
        self.pwm.start()

    def set_cycle(self, duty_cycle):
        "Configure duty cycle of PWM"
        self.pwm.ChangeDutyCycle(duty_cycle)

    def set_frequency(self, frequency):
        self.pwm.ChangeFrequency(frequency)

    def clean_up(self):
        GPIO.output(self.gpio_pin, GPIO.LOW)
        self.pwm.stop()
        GPIO.cleanup()
        
        
if __name__ == "__main__":
    gpio_pin = 12
    frequency = 10000
    duty_cycle = 50 

    electromagnet = Electromagnet(gpio_pin, frequency, duty_cycle)

    try:
        while True:
            try:
                toggle = input("on/off: ")
                if toggle == "on":
                    electromagnet.turn_on()
                else:
                    electromagnet.turn_off()
                #duty_cycle = float(input("Enter duty cycle (0 to 100): "))
                #frequency = float(input("Enter frequency (Hz)): "))
                #electromagnet.set_cycle(duty_cycle)
                #electromagnet.set_frequency(frequency)
            except ValueError:
                pass
    except KeyboardInterrupt:
        pass
    electromagnet.clean_up()
