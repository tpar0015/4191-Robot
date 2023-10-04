from MotorControl.motorctl_new import Motor
from MotorControl.drive_test_final import Drive

## Check if camera see the package
## find the turning angle
## turn the robot
## turn off the magnet to shoot

# check if the percel is placed
def camera_output(image):
    
    """
    
    code from Brian
    
    """

    #
    return color_sticker    # red(-1) for left target, blue(0) for center target and green(1) for right target


def turning_angle(robot_pose, color_sticker):
    # robot_pose = -1(left), 0(center), 1(right)
    # color_sticker = -1(red), 0(blue), 1(green)
    # color_sticker - robot_pose = 0 (don't turn), < 0 (turn right), > 0 (turn left)
    angle_difference = (robot_pose - color_sticker)*15  # need to change the angle difference between targets
    return angle_difference



def aiming(magnet, motor, robot_control, robot_pose, color_sticker):

    # red(-1) for left target, blue(0) for center target and green(1) for right target
    angle_difference = turning_angle(robot_pose, color_sticker)
    robot_control.drive_deg(angle_difference,right_left = 1)

    sleep(5)
    # Turn off
    magnet.turn_off()



    
if __name__ == '__main__':
    magnet = Electromagnet()
    motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
    robot_control = Drive([0,0,np.pi/2])
    aiming(magnet, motor, robot_control, robot_pose, color_sticker)