# Module code for Electromagnet
import RPi.GPIO as GPIO
import time
import sys
from pins import *
from MotorControl.motorctl_new import Motor

class Electromagnet:
    """
    Class module to control the electromagnet
    Functions include: Toggling the electromagnet
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
    try:

        gpio_pin = 26
        frequency = 10000
        duty_cycle = 50 

        electromagnet = Electromagnet(gpio_pin, frequency, duty_cycle)
        drawback_motor = Motor(6, 18, 23)
        x = True
        while x:
            s = input('Forward done?  ')
            if s == 'y':
                x = False
            else:
                drawback_motor.set_speed(100)
                drawback_motor.forward()
                time.sleep(0.5)
                drawback_motor.stop()
        y = True
        while y:
            s = input("Drawback done? ")
            if s == 'y':
                y = False
            else:
                drawback_motor.set_speed(100)
                drawback_motor.backward()
                time.sleep(0.5)
                drawback_motor.stop()
        input('Release')
        electromagnet.turn_on()
        input('done')
        electromagnet.clean_up()
    except:
        electromagnet.clean_up()
        # try:
    #     print("Going Forward")
    #     while True:
    #         toggle = input("Done? (y/n)")
    #         if toggle == "y":
    #             break
    #         forward_time = float(input("Forward Time: "))
    #         drawback_motor.set_speed(100)
    #         drawback_motor.forward()
    #         time.sleep(forward_time)
    #         drawback_motor.stop()

            
    #     while True:
    #         try:
    #             toggle = input("on/off: ")
    #             if toggle == "on":
    #                 electromagnet.turn_on()
    #                 while True:
    #                     x = input("Drawback Time: ")
    #                     x = float(x)
    #                     print(x)
    #                     drawback_motor.set_speed(100)
    #                     drawback_motor.backward()
    #                     time.sleep(x)
    #                     drawback_motor.stop()
    #                     toggle = input("Done? (y/n)")
    #                     if toggle == "y":
    #                         electromagnet.turn_off()
    #                         break
    #             else:
    #                 electromagnet.turn_off()
    #             #duty_cycle = float(input("Enter duty cycle (0 to 100): "))
    #             #frequency = float(input("Enter frequency (Hz)): "))
    #             #electromagnet.set_cycle(duty_cycle)
    #             #electromagnet.set_frequency(frequency)
    #         except ValueError:
    #             GPIO.cleanup()
    #             pass
    # except KeyboardInterrupt:
    #     pass
    # electromagnet.clean_up()
