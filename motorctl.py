from enum import Enum
from gpiozero import PWMLED
from time import sleep



Dir = Enum('Dir, ['FORWARD', 'REVERSE']')

# Set parameters
pwm_freq = 18000 # Hz (20kHz max)
direction = Dir.FORWARD
speed = 100 # 0 to 100 (%)


# Init pins and PWM

OUT_A = PWMLED(12, frequency=pwm_freq)
OUT_B = PWMLED(13, frequency=pwm_freq)

OUT_A.value = speed
OUT_B.value = speed

# TODO: Logic to set pins based on direction
#? docs: https://www.pololu.com/product/2997
