# All is implemented according https://www.pololu.com/product/2997
import RPi.GPIO as GPIO
import time

# init pins (can be changed)

# Wheel 1(should be left)
IN_LF = 12
IN_LB = 15
EN_L = 13

# Wheel 2(should be right)
IN_RF = 32
IN_RB = 36
EN_R = 31


GPIO.setmode(GPIO.BOARD)  # can be changed to BCM


class Motor:
    """
    This is class to control motor(wheel) in the robot
    """

    def __init__(self):
        # make global speed control for PWM
        # FIRST ONE
        global pwm_LF
        global pwm_LB
        # SECOND ONE
        global pwm_RF
        global pwm_RB
        # can set to high initially as others is low
        GPIO.setup(IN_LF, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN_LB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN_RF, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN_RB, GPIO.OUT, initial=GPIO.HIGH)
        # set freq is 2000Hz
        pwm_LF = GPIO.PWM(IN_LF, 2000)
        pwm_LB = GPIO.PWM(IN_LB, 2000)
        pwm_RF = GPIO.PWM(IN_RF, 2000)
        pwm_RB = GPIO.PWM(IN_RB, 2000)
        # START
        pwm_LF.start(0)
        pwm_LB.start(0)
        pwm_RF.start(0)
        pwm_RB.start(0)

        # set all other pins as low(H/L Pin)
        GPIO.setup(EN_L, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(EN_R, GPIO.OUT, initial=GPIO.LOW)

    def forward(self, left_speed, right_speed):
        # LEFT WHEEL
        GPIO.output(EN_L, GPIO.HIGH)
        pwm_LF.ChangeDutyCycle(left_speed)
        pwm_LB.ChangeDutyCycle(0)

        # LEFT WHEEL
        GPIO.output(EN_R, GPIO.HIGH)
        pwm_RF.ChangeDutyCycle(right_speed)
        pwm_RB.ChangeDutyCycle(0)

    def reverse(self, left_speed, right_speed):
        # LEFT WHEEL
        GPIO.output(EN_L, GPIO.HIGH)
        pwm_LF.ChangeDutyCycle(0)
        pwm_LB.ChangeDutyCycle(left_speed)

        # LEFT WHEEL
        GPIO.output(EN_R, GPIO.HIGH)
        pwm_RF.ChangeDutyCycle(0)
        pwm_RB.ChangeDutyCycle(right_speed)

    def spin_left(self, speed):
        # Left
        GPIO.output(EN_L, GPIO.HIGH)
        pwm_LF.ChangeDutyCycle(0)
        pwm_LB.ChangeDutyCycle(speed)

        # right
        GPIO.output(EN_R, GPIO.HIGH)
        pwm_RF.ChangeDutyCycle(speed)
        pwm_RB.ChangeDutyCycle(0)

    def spin_right(self, speed):
        # Left
        GPIO.output(EN_L, GPIO.HIGH)
        pwm_LF.ChangeDutyCycle(speed)
        pwm_LB.ChangeDutyCycle(0)

        # right
        GPIO.output(EN_R, GPIO.HIGH)
        pwm_RF.ChangeDutyCycle(0)
        pwm_RB.ChangeDutyCycle(speed)

    def turn_off(self):
        pwm_RF.ChangeDutyCycle(0)
        pwm_LB.ChangeDutyCycle(0)
        pwm_RB.ChangeDutyCycle(0)
        pwm_LF.ChangeDutyCycle(0)


if __name__ == "__main__":
    motor = Motor()
    motor.forward(100, 100)
    time.sleep(2)
    motor.turn_off()
    GPIO.cleanup()
