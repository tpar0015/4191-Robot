from gpiozero import LED
import time

# this is LED py file
# LED lighting condition:
# 1. Camera/ultasonic sensor detect obstacles

light_pin = 8


class ledCRL:
    """
    Module for LED
    """

    def __init__(self):
        self.led = LED(light_pin)

    def light_on(self):
        self.led.on()

    def light_off(self):
        self.led.off()
