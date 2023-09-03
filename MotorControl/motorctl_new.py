import sys
import time
import RPi.GPIO as GPIO
sys.path.append("/home/tom/4191-Robot/")
from pins import *
from rotary_new import RotaryEncoder

class Motor():
    """Module for controlling a single motor"""
    def __init__(self, enable_pin, pin_a, pin_b, speed=100):
        self.enable_pin = enable_pin
        self.pin_a = pin_a
        self.pin_b = pin_b

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.pin_a, GPIO.OUT)
        GPIO.setup(self.pin_b, GPIO.OUT)

        self.speed = speed
        self.pwm = GPIO.PWM(self.enable_pin, 100)
        self.pwm.start(speed)

    def forward(self):
        """Moves motor forward"""
        GPIO.output(self.pin_a, GPIO.HIGH)
        GPIO.output(self.pin_b, GPIO.LOW)


    def backward(self):
        """Moves motor backward"""
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.HIGH)

    def stop(self):
        """Stops motor"""
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.LOW)

    def set_speed(self, speed):
        """Sets motor speed"""
        self.speed = speed
        self.pwm.ChangeDutyCycle(speed)

if __name__=="__main__":
    right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
    left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])
    left_motor.forward()
    time.sleep(5)
    left_motor.stop()
    GPIO.cleanup()