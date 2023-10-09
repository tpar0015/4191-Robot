## Main running loop for the package delivery robot
## Implements a state machine with two states: 'Load' and 'Fire'
from electromagnet import Electromagnet
from MotorControl.motorctl_new import Motor
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.drive_test_final import Drive
from multiprocessing import Queue, Process
from pins import *
from load import load
from time import sleep
from aiming import aiming, turning_angle
import numpy as np

"""
Main Procedure:
1. Load Robot 
2. Put colour in front to detect Colour
3. Use colour to determine pullback and aiming angle
4. Use motor and electromagnet to drawback
5. Pivot to angle
6. Shoot
7. Repeat from 1
"""


# initialise electromagnet, electromagnet drawback motor
mag = Electromagnet()
launcher = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)

# initialise wheels
#wheel_left = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"], speed=100)
#wheel_right = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"], speed=100)
encoder_left = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
encoder_right = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"])

# Package 1 (smallest): Red
# Package 2 (middle): Green
# Package 3 (large): Blue

"""
Aiming
robot_pose = -1(left), 0(center), 1(right)
color_sticker = -1(red), 0(blue), 1(green)
color_sticker - robot_pose = 0 (don't turn), < 0 (turn right), > 0 (turn left)
"""
robot_pose = [0,0,0]

###
while True:
    # Check current colour for destination
    color_sticker = #get colour output
    ##! Load cycle
    load(mag, launcher)
    
    ##! Fire cycle
    # Orient toward the target
    motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
    test_queue = Queue()
    robot_control = Drive(robot_pose, test_queue)
    
    aiming(mag, motor, robot_control, robot_pose, color_sticker)

    # Reset
    # Change current pose depending on sticker
    robot_pose = [0,0,color_sticker]
    sleep(10)