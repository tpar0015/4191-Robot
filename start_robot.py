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
from MotorControl.drive_new import Drive
# Sensors
from ultrasonic import Ultrasonic
from camera import capture
from detector import detect_obstacle

# Localisation, Path Planning & Navigation
from Navigation.graph import Node, Graph
from Navigation.mapping import Map

# Other 
from led import ledCRL

import multiprocessing
from multiprocessing import Process, Value, Array, Queue
#print("Number of cpu: ", multiprocessing.cpu_count())
import RPi.GPIO as GPIO
import argparse
import ast


class Robot_Controller:
    def __init__(self):
        # Initialize Queues
        self.ultrasonic_queue = Queue()
        self.drive_queue = Queue()
        self.map_queue = Queue()

        ## Define classes
        # LED 
        self.led = ledCRL() # If not used in ultrasonic already

        # Initialise map
        start_pose = [0, 0, 0]
        self.map = Map((1200,1200), 20, start_pose)
        self.map.generate_map()
        self.pose = start_pose
        # Initialise robot
        self.Control = Drive(start_pose, self.drive_queue)
        # Initialize Ultrasonic
        self.ultrasonic_proc = multiprocessing.Process(target=self.run_ultrasonic, args=())
        self.ultrasonic_proc.start()
        self.detect_distance = 150   # mm    

        # Initialize Drive Process
        self.drive_proc = None

    def run_ultrasonic(self):
        ultrasonic = Ultrasonic(self.ultrasonic_queue)
        ultrasonic.setup()
        ultrasonic.loop()

    def get_queue(self, queue):
        if queue.empty():
            return None
        else: 
            return queue.get_nowait()

    def drive_to_waypoint(self, waypoint):
        # Calculate path
        self.map.update_path(waypoint)
        path = self.map.get_path_xy()
        maps = [self.map]
        # Drive to each node
        node_idx = 0
        while node_idx < len(path):
            pose_val = self.get_queue(self.drive_queue)
            pose = pose_val if pose_val is not None else self.pose
            # Updates location
            self.map.update_location(pose)

            ultrasonic_readout = self.get_queue(self.ultrasonic_queue)
            if self.ultrasonic_queue is not None:            
                remapped_bool = self.map.check_obstacle(ultrasonic_readout, self.detect_distance)    # Checks obstacle, if obstacle remaps
            else:
                remapped_bool = False
            if remapped_bool:
                path = self.map.get_path_xy()    # Updates path
                node_idx = 0
                # For testing
                maps.append(self.map)
                continue    # Skips rest of iteration

            x,y = path[node_idx]
            self.drive_proc = Process(target=self.Control.drive_to_point, args=(x, y, theta_end=np.pi/2))
            self.drive_proc.start()
            self.pose = self.Control.get_pose()
            print(f"Arrived at Node: {path[node_idx]}, Current Position: {pose}")
            time.sleep(5)
        return maps     # Returns list of maps for testing
    def multiple_waypoints(self, waypoints):
        for waypoint in waypoints:
            self.drive_to_waypoint(waypoint)

    def terminate_processes(self):
        self.ultrasonic_proc.terminate()
        self.ultrasonic_proc.join()
        if self.drive_proc is not None:
            self.drive_proc.terminate()
            self.drive_proc.join()
        self.Control.left_motor.stop()
        self.Control.right_motor.stop()
        self.Control.left_encoder.reset_count()
        self.Control.right_encoder.reset_count()
        GPIO.cleanup()
    ################################################################
# Functions for multiprocessing
# Ultrasonic

if __name__ == "__main__":
    Robot = Robot_Controller()
    try:
        maps = Robot.drive_to_waypoint([500, 500])
    except KeyboardInterrupt:
        Robot.terminate_processes()

    for map in maps:
        map.draw_map(draw_path=True)
    # # Initialise args
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
        # time.sleep(10)
        

        

        
        

        
    

    