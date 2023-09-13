import sys
import time
import RPi.GPIO as GPIO
from multiprocessing import Queue, Process
import numpy as np
import math
sys.path.append("/home/tom/4191-Robot/")
from ultrasonic import Ultrasonic
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.motorctl_new import Motor
from pins import *

class Drive:
    def __init__(self, pose, ultrasonic_queue):
        self.right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"]) # 设置 motor
        self.left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])

        self.right_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"]) # 设置encoder
        self.left_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"] )
        print("Initialized Encoders")
        self.ultrasonic_queue = ultrasonic_queue
        self.total_ticks = 0    # ticks 和 radius
        self.turn_radius = 110

        self.wheel_radius = 0.0524 * 1000  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.speed = 45 # 1. reduce current speed to 50
        self.pose = pose

        self.ultrasonic_proc = Process(target=self.run_ultrasonic, args=())
        self.ultrasonic_proc.start()
        self.detect_distance = 149   # mm    


        self.break_flag = False
    def run_ultrasonic(self):
        ultrasonic = Ultrasonic(self.ultrasonic_queue)
        ultrasonic.setup()
        ultrasonic.loop()
    
    def set_speed(self, speed):
        self.speed = speed

    def turn(self, theta):
        """Turns robot left by theta radians"""
        self.stops_by_speed()
        # Calculate number of ticks to turn
        turn_distance = abs(theta) * self.turn_radius
        num_ticks = turn_distance / self.distance_per_tick

        if theta > 0:
            left_speed = -self.speed
            right_speed = self.speed
        else:
            left_speed = self.speed
            right_speed = -self.speed

        # Do Turn
        self.control(num_ticks, left_speed, right_speed)
        
        self.left_motor.stop()
        self.right_motor.stop()

        # Check Ticks
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        sum_ticks = left_ticks + right_ticks
        # Update Pose
        if theta > 0:
            self.pose[2] += sum_ticks * self.distance_per_tick / self.turn_radius
        else:
            self.pose[2] -= sum_ticks * self.distance_per_tick / self.turn_radius

# Forward Testing results:
# 1m = 9000
        
    def drive_forward(self, distance):
        """Drives robot forward by distance"""
        assert distance > 0, "Distance must be positive"
        # Stop Motors
        self.stops_by_speed()
        # Set forward both motors
        left_speed, right_speed = self.speed, self.speed
        # Calculate number of ticks to drive
        num_ticks = 2 * distance / self.distance_per_tick
        print("====Forward====")
        print("sum_ticks = ", self.left_encoder.count + self.right_encoder.count)
        print("Drive Ticks: ", num_ticks)
        # time.sleep(2)
        # Run Motors
        self.control(num_ticks, left_speed, right_speed)
        self.left_motor.stop()
        self.right_motor.stop()
        # Check Ticks
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        # Update Pose
        tick_sum = left_ticks + right_ticks
        measure_distance = tick_sum * self.distance_per_tick / 2
        # Print
        print("=== Forward - Update Pose ===")
        print("Theta: ", self.pose[2])
        # time.sleep(2)
        self.pose[0] += measure_distance * math.cos(self.pose[2])
        self.pose[1] += measure_distance * math.sin(self.pose[2])

    def drive_backward(self, distance, pid):
        """Drives robot backward by distance"""
        
        assert distance > 0, "Distance must be positive"
        self.stops_by_speed()
        time.sleep(0.1)

        # Set backward both motors
        left_speed, right_speed = -self.speed, -self.speed

        num_ticks = distance / self.distance_per_tick

        self.control(num_ticks, left_speed, right_speed)
        self.left_motor.stop()
        self.right_motor.stop()

        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count

        tick_sum = left_ticks + right_ticks
        measure_distance = tick_sum * self.distance_per_tick / 2

        self.pose[0] -= measure_distance * math.cos(self.pose[2])
        self.pose[1] -= measure_distance * math.sin(self.pose[2])

    def stops_by_speed(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def reset_encoders(self):
        self.left_encoder.reset_count()
        self.right_encoder.reset_count()

    def degrees_to_range(self,degrees):
        degrees = degrees % (2*np.pi)  # type: ignore # Ensure degrees are within 0 to 359
        if degrees > np.pi:
            degrees -= 2*np.pi  # Convert to the range -180 to +179
        return degrees

    def drive_forward_tick(self,ticks):
        self.right_motor.set_speed(self.speed+2.5) # Adjust the speed of right motor (motor speed uncertainty)
        self.left_motor.set_speed(self.speed)
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        print("Initial left ticks: ", left_ticks, " right ticks", right_ticks,"\n")
        print("Drive Forward Ticks: ", ticks)
        time.sleep(2)
        # ultrasonic check
            
        while (left_ticks<ticks or right_ticks <ticks):
            if(left_ticks>ticks):
                self.left_motor.stop()
            if(right_ticks>ticks):
                self.right_motor.stop()
            left_ticks = self.left_encoder.count
            right_ticks = self.right_encoder.count
            print("Cur left ticks: ", left_ticks, " right ticks", right_ticks,"\n")
        
        self.stops_by_speed()
        self.reset_encoders()
    
    def drive_turn_tick(self,ticks,left_right):
        if(left_right==1):
            # LEFT
            self.right_motor.set_speed(self.speed+2.5) # Adjust the speed of right motor (motor speed uncertainty)
            self.left_motor.set_speed(-self.speed)
        else:
            # RIGHT
            self.right_motor.set_speed(-self.speed-2.5) # Adjust the speed of right motor (motor speed uncertainty)
            self.left_motor.set_speed(self.speed)
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        print("Initial left ticks: ", left_ticks, " right ticks", right_ticks,"\n")

        while (abs(left_ticks)<ticks or abs(right_ticks) <ticks):
            if(left_ticks>ticks):
                self.left_motor.stop()
            if(right_ticks>ticks):
                self.right_motor.stop()
            left_ticks = self.left_encoder.count
            right_ticks = self.right_encoder.count
            print("Cur left ticks: ", left_ticks, " right ticks", right_ticks,"\n")
        
        self.stops_by_speed()
        self.reset_encoders()


    # passing distance in m
    def drive_dis(self, distance):
        ticks_forward = round(distance*9000)
        self.drive_forward_tick(ticks_forward)


    def drive_deg(self, deg,right_left):
        deg_ticks = 1550 * deg/(np.pi/2)
        right_left = 1 # left
        if(deg_ticks>0):
            right_left = 0 # right
        self.drive_turn_tick(abs(deg_ticks),right_left)
        self.pose[2] -= deg
        self.pose[2]  = self.degrees_to_range(self.pose[2]) # convert to accurate range

    def control(self, num_ticks, left_speed, right_speed):
        """Drives motors for num_ticks at speed, with PID tuning"""
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        print("===Control===")
        print(f"left_ticks: {left_ticks}, right_ticks: {right_ticks}")

        print("Number of Ticks: ", num_ticks)
        print("Starting...")

        

        total_ticks_start = self.get_total_ticks()
        # While num_ticks not reached
        while self.get_total_ticks() < num_ticks + total_ticks_start - 60:
            self.get_total_ticks()
            diff_ticks = self.left_encoder.count - self.right_encoder.count
            print(f"Left Ticks: {self.left_encoder.count}, Right Ticks: {self.right_encoder.count}")
            
            self.right_motor.set_speed(right_speed)
            self.left_motor.set_speed(left_speed)
            self.update_total_ticks()

    # Logic:
    # Check if robot need to turn
    # turn first
    # Then forward

    def drive_to_point(self, x, y, theta_end=None):
        """Drives robot to point (x, y)"""
        print("Pose: ", self.pose)
        dx = x - self.pose[0]
        dy = y - self.pose[1]
 
        #theta = math.atan2(dy, dx) 
        # # the angle between cur_poos to des
        theta = math.atan2(dy, dx)
        print("\n initial check : ", "dx:  ", dx, "dy: " , dy, "theta : ", theta)
        print("\n curent robot orientation: ", self.pose[2])
        rot_deg = self.pose[2] - theta
        print("\n rot deg: ", rot_deg)
        rot_deg = self.degrees_to_range(rot_deg)
        print("\n after adj : ", rot_deg)


        self.drive_deg(rot_deg,None)

        distance = math.sqrt(dx**2 + dy**2)
        distance = distance*1e-3 # Convert to mm

        if distance < 0.02:
            print("At Point")
            return

        self.drive_dis(distance)
        self.pose[:2] = [x,y]
        # if theta_end is not None:
        #     # Difference in angle between current and desired
        #     theta_diff    # # Initialise args
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--wayp0", type=str, default='[-1,-1]')
    # parser.add_argument("--wayp1", type=str, default='[-1,-1]')
    # parser.add_argument("--wayp2", type=str, default='[-1,-1]')
    # parser.add_argument("--pose", type=str, default='[-1,-1,-1]')
    # args, _ = parser.parse_known_args()

    # # Initial positions absolute (in cm (x,y))
    # # If no inputs, using sample arena waypoints, change for final pls
    # if args.wayp0 == '[-1,-1]' or args.wayp1 == '[-1,-1]' or args.wayp0 == '[-1,-1]':
    #     wayp_0 = [30, 20]
    #     wayp_1 = [90, 80]
    #     wayp_2 = [30, 80]
    # else:
    #     wayp_0 = ast.literal_eval(args.wayp0)
    #     wayp_1 = ast.literal_eval(args.wayp1)
    #     wayp_2 = ast.literal_eval(args.wayp2)

    # if args.pose == '[-1,-1,-1]':
    #     robot_pose = [30, 20, 0] # x, y, pose
    # else:
    #     robot_pose = ast.literal_eval(args.pose)

    # wayp_all = [wayp_0, wayp_1, wayp_2]
    # ################################################################################
    # #arr1 = Array('i', range(10))
    # #proc1 = Process(target=function, args=(arr1,))
    # #proc1.start()
    # #proc1.join()
    # #multiprocessing.Process(target=drive_to_waypoint, args=())
    # # Move to waypoints
    # for curr in range(len(wayp_all)):
    #     """
    #     Sample robot procedure:
    #     1. Detect obstacles
    #     2. Path planning
    #     3. Robot movement
    #     """
    #     ## Detect obstacles to update map

    #     # TODO Use ultrasonic sensor to update map
    #     ## Path Planning
    #     map.update_path(wayp_all[curr])
    #     path = map.get_path_xy()
    #     #shared_array = multiprocessing.Array('i', array_size)

    #     # Take a picture with camera and save it
    #     # TODO Potential replace camera taking with real-time
    #     # camera_proc = multiprocessing.Process(target=capture, args=(1,"data/scene.jpg"))
    #     # camera_proc.start()
    #     # camera_proc.join()


        ###  Main Loop 

        ## Robot movement
        # Move to target waypoint
        # speed = 1
        # angle = robot_pose[2]
        # position = [robot_pose[0],robot_pose[1]]
        # waypoints = [start_node, target_node]

        # drive_proc.join() # Wait for drive process to finish
        # # Wait 10 seconds at/near waypoint to check
        # time.sleep(10) = theta_end - self.pose[2]
        #     if theta_diff > np.pi/16:
        #         self.turn(theta_diff)


    def drive_to_waypoints(self, waypoints:list):
        for waypoint in waypoints:
            self.drive_to_point(waypoint[0],waypoint[1]) # TODO: can change the theta end


    def get_pose(self):
        return self.pose

    def clean(self):
        GPIO.cleanup()

# Forward Testing results:
# 1m = 9000
# 1550 = 90deg
# 1 == left 0 == right
if __name__== "__main__":
    test_queue = Queue()
    robot_control = Drive([0,0,np.pi/2], test_queue)
    try: 
        robot_control.drive_to_point(600,600)
        time.sleep(12)
        robot_control.drive_to_point(0, 500)
        #     # input("drive tes")
        #     # print("turning 90degs left\n")
        #     # time.sleep(0.5)

        #     robot_control.drive_dis(0.4)
        #     #robot_control.wheelCalibration_forward(3600)
        #     print("40cm forward \n")
        #     time.sleep(0.5)
        #     robot_control.drive_deg(90,0)
        #     #robot_control.wheelCalibration_turning(1550,0)
        #     print("turning 90degs right\n")
        #     time.sleep(0.5)

        #     robot_control.drive_dis(0.8)
            
        #     #robot_control.wheelCalibration_forward(7200)
        #     print("80cm forward \n")
        #     time.sleep(0.5)

        #     input("check")
        # # moving calibration


        # # theta = -np.pi
        # # robot_control.turn(theta)
        # # print("second round")
        # # time.sleep(2)
        
        # # theta = np.pi
        # # robot_control.turn(theta)

        

        


        # robot_control.drive_to_point(200,200,None)
        # time.sleep(5)
        # robot_control.drive_to_point(200,300, None)
        # robot_control.drive_forward(50, pid_forward)
        # robot_control.turn(-np.pi/4, pid_turn)
        # robot_control.turn(np.pi/2)
        
        print("Done")
    except Exception as e:
        print(e.args)
    #     robot_control.left_motor.stop()
    #     robot_control.right_motor.stop()
    #     GPIO.cleanup()
    GPIO.cleanup()


