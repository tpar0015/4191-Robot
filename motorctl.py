from enum import Enum
Dir = Enum('Dir, ['FORWARD', 'REVERSE']')

# Set parameters
pwm_freq = 18000 # Hz (20kHz max)
direction = Dir.FORWARD
speed = 100 # 0 to 100 (%)

#? TODO: Set pin values and PWM

