## Main running loop for the package delivery robot
## Implements a state machine with two states: 'Load' and 'Fire'

from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from MotorControl.rotary_new import RotaryEncoder
from pins import *
from load import load



# initialise electromagnet, electromagnet drawback motor
mag = Electromagnet()
launcher = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)

# initialise wheels
wheel_left = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)
wheel_right = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"], speed=100)
encoder_left = RotaryEncoder(PINS[encoder1_a], PINS[encoder1_b])
encoder_right = RotaryEncoder(PINS[encoder2_a], PINS[encoder2_b])


##! Load cycle
load(mag, launcher)


##! Fire cycle
# Check colour of sticker

# Orient toward the target

# Release electromagnet