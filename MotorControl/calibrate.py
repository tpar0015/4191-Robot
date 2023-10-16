"""Calculates distance per tick"""
import math
import time
from MotorControl.motorctl_new import Motor
from pins import *
import RPi._GPIO as GPIO
from processing import MultiProcess
from multiprocessing import Manager


class Calibrate:
    def __init__(self):
        self.manager = Manager()
        self.dist_per_tick = (50 * 2 * math.pi) / (74.83 * 48)  # Distance per tick in mm
        self.wheel_radius = 50
        self.processes = MultiProcess(self.manager, ["front", "left", "right"])
        self.processes.start_processes()
        self.right_motor = Motor(PINS["motor1_en"], PINS["motor1_a"], PINS["motor1_b"])
        self.left_motor = Motor(PINS["motor2_en"], PINS["motor2_a"], PINS["motor2_b"])

    def loop(self):
        print("Calibrating distance per tick...")
        cond = True
        while cond:
            time_to_drive = input("Enter time to drive (s): ")
            time_to_drive = float(time_to_drive)
            cur_time = time.time()
            while time.time() - cur_time < time_to_drive:
                self.right_motor.set_speed(100)
                self.left_motor.set_speed(100)

            self.right_motor.stop()
            self.left_motor.stop()
            y_n = input("Did the robot drive 1m? (y/n): ")
            if y_n == 'y':
                cond = True
            else:
                self.processes.reset_count()
        



if __name__=="__main__"
    calibration = Calibrate()
    GPIO.cleanup()
                

