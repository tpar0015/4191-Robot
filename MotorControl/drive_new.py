import sys
import time
import RPi.GPIO as GPIO

import numpy as np
import math
sys.path.append("/home/tom/4191-Robot/")
from MotorControl.rotary_new import RotaryEncoder
from MotorControl.motorctl_new import Motor
from pins import *
from multiprocessing import Queue
class Drive:
    def __init__(self, pose, pid_forward, pid_turn, queue: Queue = None):
        self.right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
        self.left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])
        self.right_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
        self.left_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"] )
        print("Initialized Encoders")
        self.turn_radius = 121
        # self.wheel_radius = 50
        # self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in mm
        self.wheel_radius = 0.0524 * 1000  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.pid_forward = pid_forward
        self.pid_turn = pid_turn
        self.speed = 100
        self.pose = pose
        self.Kp = 0.65
        self.Ki = 0.01
        self.Kd = 0.01

        if queue is not None:
            self.queue = queue
        else:
            self.queue = Queue()
    def set_speed(self, speed):
        self.speed = speed

    def turn(self, theta, pid):
        """Turns robot left by theta radians"""
        self.stops_by_speed()
        self.reset_encoders()
        # Calculate number of ticks to turn
        turn_distance = abs(theta) * self.turn_radius
        num_ticks = turn_distance / self.distance_per_tick

        if theta > 0:
            self.speed = -self.speed
        # Do Turn
        self.control(num_ticks, self.speed, pid)
        
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

        
    def drive_forward(self, distance, pid):
        """Drives robot forward by distance"""
        assert distance > 0, "Distance must be positive"
        # Reset encoders and stop motors
        self.reset_encoders()
        self.stops_by_speed()
        time.sleep(1)

        # Set forward mode
        self.left_motor.forward()
        self.right_motor.forward()

        # Calculate number of ticks to drive
        num_ticks = 2 * distance / self.distance_per_tick
        print("====Forward====")
        print("sum_ticks = ", self.left_encoder.count + self.right_encoder.count)
        print("Drive Ticks: ", num_ticks)
        time.sleep(2)
        # Run Motors
        self.control(num_ticks, self.speed, pid_forward)
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
        time.sleep(0.1)
        self.reset_encoders()
        self.stops_by_speed()

        self.left_motor.backward()
        self.right_motor.backward()


        num_ticks = distance / self.distance_per_tick

        self.control(num_ticks, self.speed, pid)
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

    def control(self, num_ticks, speed, pid):
        """Drives motors for num_ticks at speed, with PID tuning"""
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        print("===Control===")
        print(f"left_ticks: {left_ticks}, right_ticks: {right_ticks}")
        time.sleep(2)
        print("Number of Ticks: ", num_ticks)
        print("Starting...")
        time.sleep(1)
        diff_ticks = self.left_encoder.count - self.right_encoder.count
        Kp, Ki, Kd = pid
        P = 0
        I = 0
        D = 0
        prev_diff_ticks = 0
        print(self.left_encoder.count, self.right_encoder.count)
        while self.left_encoder.count + self.right_encoder.count < num_ticks:
            print("in loop")
            diff_ticks = self.left_encoder.count - self.right_encoder.count
            print(f"Left Ticks: {self.left_encoder.count}, Right Ticks: {self.right_encoder.count}")
            print("Diff Ticks: ", diff_ticks)
            if diff_ticks != 0:
                P = Kp * abs(diff_ticks)/abs(diff_ticks)
                I += Ki * abs(diff_ticks)/abs(diff_ticks)
                D = Kd * abs(diff_ticks - prev_diff_ticks)/abs(diff_ticks)
            print("P + I + D: ", P + I + D)
            sum_pid = min(P + I + D, 100)
            tune = 1 - sum_pid/100
            print("tune: ", tune)
            if diff_ticks > 0:
                # left_speed = min(speed,max(speed - math.floor(diff_ticks / (2 / self.Kp)),0))
                left_speed = tune*speed
                right_speed = speed
            elif diff_ticks < 0:
                left_speed = speed
                right_speed = tune*speed
                # right_speed = min(max(speed - math.ceil(diff_ticks / (2 / self.Kp)),0), speed)
                # right_speed = max(min(P + I + D, 100), 0)
            else:
                left_speed = speed
                right_speed = speed
            if left_speed >= 0:
                self.left_motor.forward(left_speed)
            else:
                self.left_motor.backward(left_speed)
            if right_speed >= 0:
                self.right_motor.forward(right_speed)
            else:
                self.right_motor.backward(right_speed)
            prev_diff_ticks = diff_ticks

    def drive_to_point(self, x, y, theta_end=None):
        """Drives robot to point (x, y)"""
        dx = x - self.pose[0]
        dy = y - self.pose[1]
        theta = math.atan2(dy, dx) - self.pose[2]
        print("=====Start Turn=====")
        print("===Theta===: ", theta)
        time.sleep(2)
        self.turn(theta, self.pid_turn)
        distance = math.sqrt(dx**2 + dy**2)
        if distance == 0:
            print("At Point")
            return

        time.sleep(0.1)
        self.drive_forward(distance, self.pid_forward)
        if theta_end is not None:
            # Difference in angle between current and desired
            theta_diff = theta_end - self.pose[2]
            if theta_diff > np.pi/16:
                self.turn(theta_diff, self.pid_turn)
        # Adds pose to queue
        self.queue.put(self.get_pose())
    def get_pose(self):
        return self.pose

    def clean(self):
        GPIO.cleanup()

if __name__== "__main__":
    test_queue = Queue()
    pid_forward = [1,0.01,0.01]
    pid_turn = [0.65, 0.01, 0.01]
    robot_control = Drive([0,0,np.pi/2],pid_forward,pid_turn ,test_queue)
    try:
        # robot_control.drive_to_point(200,0,None)
        # print("1st point, position: ",robot_control.pose)
        # time.sleep(5)
        # robot_control.drive_to_point(200,300, None)
        # robot_control.drive_forward(50, pid_forward)
        robot_control.turn(np.pi/4, pid_turn)
        # robot_control.turn(np.pi/2)
        
        print("Done")
    except Exception:
        robot_control.left_motor.stop()
        robot_control.right_motor.stop()
    GPIO.cleanup()


