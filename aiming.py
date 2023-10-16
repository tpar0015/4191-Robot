from MotorControl.motorctl_new import Motor
from MotorControl.drive_test_final import Drive
from electromagnet import Electromagnet
from pins import *
from time import sleep

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
    return QR_code    # -1 for left target, 0 for center target and 1 for right target


def turning_angle(robot_pose, QR_code):
    # robot_pose = -1(left), 0(center), 1(right)
    # QR_code = -1(left), 0(center), 1(right)
    # QR_code - robot_pose = 0 (don't turn), < 0 (turn left), > 0 (turn right)
    curr_pose = robot_pose[2] 
    angle_difference = (QR_code - current_pose)
        
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
    aiming(magnet, motor, robot_control, robot_pose = 0, color_sticker = 1)
