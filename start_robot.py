# ROBOT OPERATION
"""
Functions and Files:
#TODO completed
led.py - Turns on and off light
motorctl.py - Motor control

#TODO NEEDS TESTING
detector.py - Detecting bounding boxes from image 
drive.py - Determine motor inputs given pos and waypoint
rotary.py - Rotar encoder code
ultrasonic.py - Runs the ultrasonic module

#TODO IN PROGRESS
ceiling.py - Ceiling localisation code
target.py - Sample camera code


"""
# Call modules for robot
import time
import sys
import numpy as np

# Driving
from MotorControl.drive_test_final import Drive
# Sensors
from ultrasonic import Ultrasonic
# from camera import capture
# from detector import detect_obstacle

# Localisation, Path Planning & Navigation
from Navigation.graph import Node, Graph
from Navigation.mapping import Map

# Other 
# from led import ledCRL

import multiprocessing
from multiprocessing import Process, Value, Array, Queue
#print("Number of cpu: ", multiprocessing.cpu_count())
import RPi.GPIO as GPIO
import argparse
import ast


class Robot_Controller():
    def __init__(self, start_pose):
        GPIO.setmode(GPIO.BCM)
        # Initialize Queues
        self.ultrasonic_queue = Queue()
        self.drive_queue = Queue()
        self.map_queue = Queue()

        ## Define classes
        # LED 
        # self.led = ledCRL() # If not used in ultrasonic already

        # Initialise map
        self.map = Map((1200,1200), 50, start_pose)
        self.map.generate_map()
        self.pose = start_pose
        # Initialise robot
        self.Control = Drive(self.pose,self.ultrasonic_queue)
        
        # Initialize Ultrasonic]
        self.ultrasonic_proc = multiprocessing.Process(target=self.run_ultrasonic, args=())
        self.ultrasonic_proc.start()
        self.detect_distance = 150   # mm    

        # Initialize Drive Process
        self.drive_proc = None

    def run_ultrasonic(self):
        ultrasonic = Ultrasonic(self.ultrasonic_queue)
        ultrasonic.setup()
        ultrasonic.loop()
    
    def drive(self, x, y, theta_end):
        self.Control.drive_to_point(x, y, theta_end)

    def get_queue(self, queue):
        if queue.empty():
            return None
        else: 
            return queue.get_nowait()

    def drive_to_waypoint(self, waypoint):
        # Calculate path
        self.map.update_path(waypoint)
        path = self.map.get_path_xy()
        input('Check')
        maps = [self.map]
        # Drive to each node
        node_idx = 1
        while node_idx < len(path):
            pose = self.pose
            # Updates location
            self.map.update_location(pose)

            ultrasonic_readout = self.get_queue(self.ultrasonic_queue)
            if ultrasonic_readout is not None:            
                remapped_bool = self.map.check_obstacle(ultrasonic_readout, self.detect_distance)    # Checks obstacle, if obstacle remaps
            else:
                remapped_bool = False
            if remapped_bool:
                print("Remapped")
                self.Control.stops_by_speed()
                path = self.map.get_path_xy()    # Updates path
                node_idx = 1
                # For testing
                maps.append(self.map)
                continue    # Skips rest of iteration

            x,y = path[node_idx]
            print("x,y", x,y)
            input('')
            theta_end = None
            self.drive(x, y, theta_end)
            self.pose = self.Control.get_pose()
            print(f"Arrived at Node: {path[node_idx]}, Current Position: {pose}")
            node_idx += 1
            time.sleep(2)
        return maps     # Returns list of maps for testing
    def multiple_waypoints(self, waypoints):
        for waypoint in waypoints:
            self.drive_to_waypoint(waypoint)

    def terminate_processes(self):
        self.Control.left_motor.stop()
        self.Control.right_motor.stop()
        self.ultrasonic_proc.terminate()
        self.ultrasonic_proc.join()
        if self.drive_proc is not None:
            self.drive_proc.terminate()
            self.drive_proc.join()
        GPIO.cleanup()
    ################################################################
# Functions for multiprocessing
# Ultrasonic

if __name__ == "__main__":
    start_pose = [50, 50, np.pi/2]
    Robot = Robot_Controller(start_pose)
    maps = [Robot.map]
    Robot.drive_to_waypoint((50, 800))
    # Robot.Control.drive_to_point(900,800)
    try:
        # Robot.multiple_waypoints(waypoints)
       pass 
    except KeyboardInterrupt:
        Robot.terminate_processes()
        GPIO.cleanup()
    for map in maps:
        print(map.get_path_xy())
        map.draw_arena(draw_path=True)
        

        

        
        

        
    

    