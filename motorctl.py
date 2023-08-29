# from enum import Enum
# from gpiozero import PWMLED
# from time import sleep


# motor_x_pin_FWD, motor_x_pin_REV = 12, 13     #? change later
# motor_y_pin_FWD, motor_y_pin_REV = 18, 19


# Dir = Enum("Dir", ["FORWARD", "REVERSE"])

# # Set parameters
# pwm_freq = 10000  # Hz (20kHz max)


# # init motor pins
# Motor_x_FWD = PWMLED(motor_x_pin_FWD, frequency=pwm_freq)
# Motor_x_REV = PWMLED(motor_x_pin_REV, frequency=pwm_freq)
# Motor_y_FWD = PWMLED(motor_y_pin_FWD, frequency=pwm_freq)
# Motor_y_REV = PWMLED(motor_y_pin_REV, frequency=pwm_freq)


# def motor_x(speed):
#     # speed is a floating point number between -1 to 1
#     speed_abs = abs(speed)
#     if speed == 0:
#         Motor_x_FWD.off()
#         Motor_x_REV.off()
#     elif speed > 0:
#         Motor_x_FWD.value = speed_abs
#         Motor_x_REV.off()
#     elif speed < 0:
#         Motor_x_FWD.off()
#         Motor_x_REV.value = speed_abs

# def motor_y(speed):
#     speed_abs = abs(speed)
#     if speed == 0:
#         Motor_y_FWD.off()
#         Motor_y_REV.off()
#     elif speed > 0:
#         Motor_y_FWD.value = speed_abs
#         Motor_y_REV.off()
#     elif speed < 0:
#         Motor_y_FWD.off()
#         Motor_y_REV.value = speed_abs



# # left wheel
# Motor_L_F = PWMLED(12, frequency=pwm_freq)  # FORWARD
# Motor_L_B = PWMLED(18, frequency=pwm_freq)  # BACKWARD

# Motor_L_F.off()
# Motor_L_B.off()
# # right wheel
# Motor_R_F = PWMLED(13, frequency=pwm_freq)  # FORWARD
# Motor_R_B = PWMLED(19, frequency=pwm_freq)  # BACKWARD

# Motor_R_F.off()
# Motor_R_B.off()
# # TODO: Logic to set pins based on direction
# # ? docs: https://www.pololu.com/product/2997


# # moving forward
# Motor_L_F.on()
# Motor_L_F.value = 0.5
# Motor_R_F.on()
# Motor_R_F.value = 0.5


#! This code is implemented with Table 2 'fast decay'
#! Will need to refactor using Table 1 'slow decay'


# All is implemented according https://www.pololu.com/product/2997
import RPi.GPIO as GPIO
import time
# init pins (can be changed)

#Wheel 1(should be left)
IN_A1=12
IN_A2=16
EN_A = 6
#Wheel 2(should be right)
IN_B1=18
IN_B2=22
EN_B = 27


GPIO.setmode(GPIO.BOARD)# can be changed to BCM

class Motor():
    """
    This is class to control motor(wheel) in the robot
    """
    def __init__(self):
        # make global speed control
        global pwm_A
        global pwm_B
        # can set to high initially as others is low
        GPIO.setup(EN_A, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(EN_B, GPIO.OUT, initial=GPIO.HIGH)
        # set freq is 2000Hz
        pwm_A = GPIO.PWM(EN_A,2000)
        pwm_B = GPIO.PWM(EN_B,2000)
        pwm_A.start(0)
        pwm_B.start(0)

        # set all other pins as low
        GPIO.setup(IN_A1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN_A2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN_B1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN_B1, GPIO.OUT, initial=GPIO.LOW)

    def forward(self,left_speed,right_speed):
        GPIO.output(IN_A1, GPIO.HIGH)
        GPIO.output(IN_A2, GPIO.LOW)

        GPIO.output(IN_B1, GPIO.HIGH)
        GPIO.output(IN_B2, GPIO.LOW)
        pwm_A.ChangeDutyCycle(left_speed)
        pwm_B.ChangeDutyCycle(right_speed)

    def reverse(self,left_speed,right_speed):
        GPIO.output(IN_A1, GPIO.LOW)
        GPIO.output(IN_A2, GPIO.HIGH)
        
        GPIO.output(IN_B1, GPIO.LOW)
        GPIO.output(IN_B2, GPIO.HIGH)
        pwm_A.ChangeDutyCycle(left_speed)
        pwm_B.ChangeDutyCycle(right_speed)

    
    def left(self,speed):
        
        GPIO.output(IN_A1, GPIO.LOW)
        GPIO.output(IN_A2, GPIO.HIGH)
        GPIO.output(IN_B1, GPIO.HIGH)
        GPIO.output(IN_B2, GPIO.LOW)
        pwm_A.ChangeDutyCycle(speed)
        pwm_B.ChangeDutyCycle(speed)

    def right(self,speed, speed):
        
        GPIO.output(IN_A1, GPIO.HIGH)
        GPIO.output(IN_A2, GPIO.LOW)
        GPIO.output(IN_B1, GPIO.LOW)
        GPIO.output(IN_B2, GPIO.HIGH)
        pwm_A.ChangeDutyCycle(speed)
        pwm_B.ChangeDutyCycle(speed)