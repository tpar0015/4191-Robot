from enum import Enum
from gpiozero import PWMLED
from time import sleep


motor_x_pin_FWD, motor_x_pin_REV = 25, 8     #? change later
motor_y_pin_FWD, motor_y_pin_REV = 7, 1


Dir = Enum("Dir", ["FORWARD", "REVERSE"])

# Set parameters
pwm_freq = 10000  # Hz (20kHz max)


# init motor pins
Motor_x_FWD = PWMLED(motor_x_pin_FWD, frequency=pwm_freq)
Motor_x_REV = PWMLED(motor_x_pin_REV, frequency=pwm_freq)
Motor_y_FWD = PWMLED(motor_y_pin_FWD, frequency=pwm_freq)
Motor_y_REV = PWMLED(motor_y_pin_REV, frequency=pwm_freq)


def motor_x(speed):
    # speed is a floating point number between -1 to 1
    speed_abs = abs(speed)
    if speed == 0:
        Motor_x_FWD.off()
        Motor_x_REV.off()
    elif speed > 0:
        Motor_x_FWD.value = speed_abs
        Motor_x_REV.off()
    elif speed < 0:
        Motor_x_FWD.off()
        Motor_x_REV.value = speed_abs

def motor_y(speed):
    speed_abs = abs(speed)
    if speed == 0:
        Motor_y_FWD.off()
        Motor_y_REV.off()
    elif speed > 0:
        Motor_y_FWD.value = speed_abs
        Motor_y_REV.off()
    elif speed < 0:
        Motor_y_FWD.off()
        Motor_y_REV.value = speed_abs



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
