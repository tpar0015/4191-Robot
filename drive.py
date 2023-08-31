# This file will provide functions to determine motor inputs
# given the robot's current position and [x,y] array of target waypoints

import numpy as np
import motorctl

Robot = motorctl.Motor()

# angle = None
# speed = None


# waypoints = []
# position = []

def drive(waypoints, position, speed, angle):
    # angle = arctan(diff_y/diff_x) adjusted for Q2, Q3 of unit circle
    diff_y = waypoints[0][1] - position[1]
    diff_x = waypoints[0][0] - position[0]
    target_angle = np.arctan2(diff_y, diff_x)

    angle_deviation = target_angle - angle

    # distance = sqrt(diff_x^2 + diff_y^2)
    distance = np.sqrt(sum(np.power([diff_x, diff_y], 2)))

    # turn to the target angle
    if angle_deviation > 0:
        # turn left to the angle
        Robot.spin_left(0.6)
        #? stop turning when angle is reached
        
    else:
        # turn right to the angle
        Robot.spin_right(0.6)
        #? stop turning when angle is reached
