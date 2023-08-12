from enum import Enum
from gpiozero import PWMLED
from time import sleep


Dir = Enum("Dir", ["FORWARD", "REVERSE"])

# Set parameters
pwm_freq = 18000  # Hz (20kHz max)
direction = Dir.FORWARD
speed = 100  # 0 to 100 (%)


# Init pins and PWM
# NOTE:
# .value is the PWM 0-1 which control the duty cycle(speed)

# left wheel
Motor_L_F = PWMLED(12, frequency=pwm_freq)  # FORWARD
Motor_L_B = PWMLED(18, frequency=pwm_freq)  # BACKWARD

Motor_L_F.off()
Motor_L_B.off()
# right wheel
Motor_R_F = PWMLED(13, frequency=pwm_freq)  # FORWARD
Motor_R_B = PWMLED(19, frequency=pwm_freq)  # BACKWARD

Motor_R_F.off()
Motor_R_B.off()
# TODO: Logic to set pins based on direction
# ? docs: https://www.pololu.com/product/2997


# moving forward
Motor_L_F.on()
Motor_L_F.value = 0.5
Motor_R_F.on()
Motor_R_F.value = 0.5
