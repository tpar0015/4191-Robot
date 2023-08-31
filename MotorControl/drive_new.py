import time
import RPi.GPIO as GPIO
from rotary_new import RotaryEncoder
from motorctl_new import Motor
import numpy as np
import math
from pins import *

class Drive:
    def __init__(self, pose):
        self.left_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
        self.right_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])
        self.left_encoder = RotaryEncoder(PINS["encoder1_a"], PINS["encoder1_b"])
        self.right_encoder = RotaryEncoder(PINS["encoder2_a"], PINS["encoder2_b"] )

        self.turn_radius = 0.121
        self.wheel_radius = 0.05
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres

        self.speed = 100
        self.pose = pose
        self.gain = 2

    def turn(self, theta):
        """Turns robot left by theta radians"""
        self.stops_by_speed()

        # Calculate number of ticks to turn
        turn_distance = abs(theta) * self.turn_radius
        num_ticks = turn_distance / self.distance_per_tick

        # Turn left
        if theta > 0:
            self.left_motor.backward()
            self.right_motor.forward()
        # Turn right
        else:
            self.left_motor.forward()
            self.right_motor.backward()

        # Do Turn
        self.control(num_ticks, self.speed)
        
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

        
    def drive_forward(self, distance):
        """Drives robot forward by distance"""
        assert distance > 0, "Distance must be positive"
        # Reset encoders and stop motors
        self.reset_encoders()
        self.stops_by_speed()

        # Set forward mode
        self.left_motor.forward()
        self.right_motor.forward()

        # Calculate number of ticks to drive
        num_ticks = distance / self.distance_per_tick
        # Run Motors
        self.control(num_ticks, self.speed)
        self.left_motor.stop()
        self.right_motor.stop()
        # Check Ticks
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        # Update Pose
        tick_sum = left_ticks + right_ticks
        measure_distance = tick_sum * self.distance_per_tick / 2

        self.pose[0] += measure_distance * math.cos(self.pose[2])
        self.pose[1] += measure_distance * math.sin(self.pose[2])

    def drive_backward(self, distance):
        """Drives robot backward by distance"""
        
        assert distance > 0, "Distance must be positive"
        self.reset_encoders()
        self.stops_by_speed()

        self.left_motor.backward()
        self.right_motor.backward()


        num_ticks = distance / self.distance_per_tick

        self.control(num_ticks, self.speed)
        self.left_motor.stop()
        self.right_motor.stop()

        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count

        tick_sum = left_ticks + right_ticks
        measure_distance = tick_sum * self.distance_per_tick / 2

        self.pose[0] -= measure_distance * math.cos(self.pose[2])
        self.pose[1] -= measure_distance * math.sin(self.pose[2])

    def stops_by_speed(self):
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)

    def reset_encoders(self):
        self.left_encoder.reset_count()
        self.right_encoder.reset_count()

    def control(self, num_ticks, speed):
        """Drives motors for num_ticks at speed, with PID tuning"""
        left_ticks = self.left_encoder.count
        right_ticks = self.right_encoder.count
        diff_ticks = left_ticks - right_ticks

        while self.left_encoder.count + self.right_encoder.count < num_ticks:
            if diff_ticks > 0:
                left_speed = max(speed - math.floor(diff_ticks / (2 / self.gain)),0)
                right_speed = speed
            elif diff_ticks < 0:
                left_speed = speed
                right_speed = max(speed - math.floor(diff_ticks / (2 / self.gain)),0)
            else:
                left_speed = speed
                right_speed = speed


            self.left_motor.set_speed(left_speed)
            self.right_motor.set_speed(right_speed)

    def drive_to_point(self, x, y, theta_end: None):
        """Drives robot to point (x, y)"""
        dx = x - self.pose[0]
        dy = y - self.pose[1]
        theta = math.atan2(dy, dx)
        self.turn(theta)
        distance = math.sqrt(dx**2 + dy**2)
        self.drive_forward(distance)
        if theta_end is not None:
            # Difference in angle between current and desired
            theta_diff = theta_end - self.pose[2]
            self.turn(theta_diff)

    def get_pose(self):
        return self.pose

if __name__== "__main__":
    robot_control = Drive([0,0,0])
    robot_control.drive_to_point(100,100, None)
    print(robot_control.get_pose())