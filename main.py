## Main running loop for the package delivery robot
## Implements a state machine with two states: 'Load' and 'Fire'

from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.drive_test_final import Drive
from pins import *
from load import load
from time import sleep
from aiming import aiming


# initialise electromagnet, electromagnet drawback motor
mag = Electromagnet()
launcher = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)

# initialise wheels
wheel_left = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)
wheel_right = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"], speed=100)
encoder_left = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
encoder_right = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"])

# Package 1 (smallest): Red
# Package 2 (middle): Green
# Package 3 (large): Blue

###
while True:
    # Check current colour for destination
    colour = get_colour()

    ##! Load cycle
    load(mag, launcher)

    ##! Fire cycle
    # Orient toward the target
    robot_control = Drive([0,0,np.pi/2])
    aiming(magnet, motor, robot_control, robot_pose = 0, color_sticker = 1)

    # Reset
    sleep(10)