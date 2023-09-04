import sys
import time
import RPi.GPIO as GPIO
sys.path.append("/home/tom/4191-Robot/")
from pins import *
from MotorControl.rotary_new import RotaryEncoder

class Motor():
    """Module for controlling a single motor"""
    def __init__(self, enable_pin, pin_a, pin_b, speed=0):
        self.enable_pin = enable_pin
        self.pin_a = pin_a
        self.pin_b = pin_b

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.pin_a, GPIO.OUT)
        GPIO.setup(self.pin_b, GPIO.OUT)

        self.speed = speed
        self.pwm = GPIO.output(self.enable_pin, GPIO.HIGH)
        self.pwm1 = GPIO.PWM(self.pin_a, 100)
        self.pwm2 = GPIO.PWM(self.pin_b, 100)
        self.pwm1.start(self.speed)
        self.pwm2.start(self.speed)
        

    # def forward(self):
    #     """Moves motor forward"""
    #     GPIO.PWM(self.pin_a, 100)
    #     GPIO.output(self.pin_b, GPIO.LOW)


    # def backward(self, ):
    #     """Moves motor backward"""
    #     pass
    def stop(self):
        """Stops motor"""
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)

    def forward(self):
        self.pwm1.ChangeDutyCycle(self.speed)
        self.pwm2.ChangeDutyCycle(0)
    def backward(self):
        self.pwm2.ChangeDutyCycle(self.speed)
        self.pwm1.ChangeDutyCycle(0)

    def set_speed(self, speed):
        self.speed = speed
        if speed >= 0:
            self.forward()
        else:
            self.backward()

if __name__=="__main__":
    right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
    left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])
    left_motor.forward(50)
    right_motor.forward(100)
    time.sleep(2)
    left_motor.stop()
    right_motor.stop()
    GPIO.cleanup()