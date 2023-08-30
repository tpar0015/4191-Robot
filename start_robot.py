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

# Driving
from motorctl import Motor
from drive import drive

# Sensors
from ultrasonic import Ultrasonic
#from camera import PICam
from detector import detect_obstacle

# Localisation, Path Planning & Navigation
from Navigation.graph import Node, Graph
from Navigation.mapping import Map

# Other 
from led import ledCRL

import multiprocessing
from multiprocessing import Process, Value, Array
#print("Number of cpu: ", multiprocessing.cpu_count())

import argparse
import ast


################################################################
# Functions for multiprocessing
# Ultrasonic
def run_ultrasonic():
    ultrasonic = Ultrasonic()
    ultrasonic.setup()
    ultrasonic.loop()



# Main loop
if __name__ == "__main__":
    # Initialise args
    parser = argparse.ArgumentParser()
    parser.add_argument("--wayp0", type=str, default='[-1,-1]')
    parser.add_argument("--wayp1", type=str, default='[-1,-1]')
    parser.add_argument("--wayp2", type=str, default='[-1,-1]')
    parser.add_argument("--pose", type=str, default='[-1,-1,-1]')

    args, _ = parser.parse_known_args()

    # Initial positions absolute (in cm (x,y))
    # If no inputs, using sample arena waypoints, change for final pls
    if args.wayp0 == '[-1,-1]' or args.wayp1 == '[-1,-1]' or args.wayp0 == '[-1,-1]':
        wayp_0 = [30, 20]
        wayp_1 = [90, 80]
        wayp_2 = [30, 80]
    else:
        wayp_0 = ast.literal_eval(args.wayp0)
        wayp_1 = ast.literal_eval(args.wayp1)
        wayp_2 = ast.literal_eval(args.wayp2)

    if args.pose == '[-1,-1,-1]':
        robot_pose = [30, 20, 0] # x, y, pose
    else:
        robot_pose = ast.literal_eval(args.pose)

    wayp_all = [wayp_0, wayp_1, wayp_2]
    ################################################################################
    #arr1 = Array('i', range(10))
    #proc1 = Process(target=function, args=(arr1,))
    #proc1.start()
    #proc1.join()
    #multiprocessing.Process(target=drive_to_waypoint, args=())
    

    ## Define classes
    # LED 
    led = ledCRL()

    # Initialise map
    map = Map((1200,1200), 20, robot_pose)
    map.generate_map()

    # Initialise robot
    Robot = Motor()
    
    # Move to waypoints
    for curr in range(len(wayp_all)-1):
        """
        Sample robot procedure:
        1. Detect obstacles
        2. Path planning
        3. Robot movement
        """
        ## Detect obstacles to update map
        ultrasonic_proc = multiprocessing.Process(target=run_ultrasonic, args=())
        ultrasonic_proc.start()

        #p1 = Process(target=map.update_map, args=(ultrasonic,))
        #p1.start()
        # Use ultrasonic sensor to update map
        
        ## Path Planning
        start_xy = wayp_all[curr]
        start_node = map.G.get_nearest_node(start_xy)
        target_xy = wayp_all[curr+1]
        target_node = map.G.get_nearest_node(target_xy)
        map.update_path(target_node)
        path = map.get_path_xy()
        #print(path, dist)


        ## Robot movement
        # Move to target waypoint
        speed = 1
        angle = robot_pose[2]
        position = [robot_pose[0],robot_pose[1]]
        waypoints = [start_node, target_node]
        drive_proc = multiprocessing.Process(target=drive, args=(waypoints, position, speed, angle))

        # Wait 10 seconds at/near waypoint to check
        time.sleep(10)

        drive_proc.join()

        

        
        

        
    

    