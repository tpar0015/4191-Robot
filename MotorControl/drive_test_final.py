import sys
import time
import RPi.GPIO as GPIO

import numpy as np
import math
sys.path.append("/home/tom/4191-Robot/")
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.motorctl_new import Motor
from pins import *

class Drive:
    def __init__(self, pose):
        self.right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"]) # 设置 motor
        self.left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])

        self.right_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"]) # 设置encoder
        self.left_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"] )
        print("Initialized Encoders")

        self.total_ticks = 0    # ticks 和 radius
        self.turn_radius = 110

        self.wheel_radius = 0.0524 * 1000  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.speed = 45 # 1. reduce current speed to 50
        self.pose = pose



    
    def update_total_ticks(self):
        self.total_ticks = self.left_encoder.count + self.right_encoder.count

    def get_total_ticks(self):
        self.update_total_ticks
        return self.total_ticks

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
        time.sleep(2)
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
        time.sleep(2)
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

    def drive_forward_tick(self,ticks):
        self.right_motor.set_speed(self.speed+2.5) # Adjust the speed of right motor (motor speed uncertainty)
        self.left_motor.set_speed(self.speed)
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        print("Initial left ticks: ", left_ticks, " right ticks", right_ticks,"\n")

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

            self.right_motor.set_speed(self.speed+2.5) # Adjust the speed of right motor (motor speed uncertainty)
            self.left_motor.set_speed(-self.speed)
        else:
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
        deg = 1550 * deg/90 
        self.drive_turn_tick(deg,right_left)
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

    def drive_to_point(self, x, y, theta_end=None):
        """Drives robot to point (x, y)"""
        dx = x - self.pose[0]
        dy = y - self.pose[1]
        
        theta = math.atan2(dy, dx) # the angle between cur_poos to des
        print("cal check : ", "dx:  ", dx, "dy: " , dy, "theta : ", theta)
        print("=====Start Turn=====")
        print("===Theta===: ", theta)
        
        input("check")
        time.sleep(2)
        if abs(theta) > np.pi/16:
            self.turn(theta)
        distance = math.sqrt(dx**2 + dy**2)
        if distance == 0:
            print("At Point")
            return
        input("Check")
        time.sleep(0.1)
        self.drive_forward(distance)
        if theta_end is not None:
            # Difference in angle between current and desired
            theta_diff = theta_end - self.pose[2]
            if theta_diff > np.pi/16:
                self.turn(theta_diff)

    def get_pose(self):
        return self.pose

    def clean(self):
        GPIO.cleanup()

# Forward Testing results:
# 1m = 9000
# 1550 = 90deg
# 1 == left 0 == right
if __name__== "__main__":

    robot_control = Drive([0,0,np.pi/2])
    try:
        while(True): 
            robot_control.drive_deg(90,1)
            print("turning 90degs left\n")
            time.sleep(0.5)

            robot_control.drive_dis(0.4)
            #robot_control.wheelCalibration_forward(3600)
            print("40cm forward \n")
            time.sleep(0.5)
            robot_control.drive_deg(90,0)
            #robot_control.wheelCalibration_turning(1550,0)
            print("turning 90degs right\n")
            time.sleep(0.5)

            robot_control.drive_dis(0.8)
            #robot_control.wheelCalibration_forward(7200)
            print("80cm forward \n")
            time.sleep(0.5)

            input("check")
        # moving calibration


        # theta = -np.pi
        # robot_control.turn(theta)
        # print("second round")
        # time.sleep(2)
        
        # theta = np.pi
        # robot_control.turn(theta)

        

        


        input("c")
        robot_control.drive_to_point(200,200,None)
        # print("1st point, position: ",robot_control.pose)
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


