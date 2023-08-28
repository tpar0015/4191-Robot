# Call modules for robot
import time
from Navigation.graph import Node, Graph
#from ultrasonic import Ultrasonic

import multiprocessing
from multiprocessing import Process, Value, Array
print("Number of cpu: ", multiprocessing.cpu_count())


# Main loop
if __name__ == "__main__":
    import argparse
    import ast
    # Initialise args
    parser = argparse.ArgumentParser()
    parser.add_argument("--wayp0", type=str, default='[-1,-1]')
    parser.add_argument("--wayp1", type=str, default='[-1,-1]')
    parser.add_argument("--wayp2", type=str, default='[-1,-1]')
    parser.add_argument("--pose", type=str, default='[-1,-1,-1]')

    args, _ = parser.parse_known_args()

    #arr1 = Array('i', range(10))
    #proc1 = Process(target=function, args=(arr1,))
    #proc1.start()
    #proc1.join()


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

    # Move to waypoints
    for curr in range(len(wayp_all)-1):
        """
        Sample robot procedure:
        1. Detect obstacles
        2. Path planning
        3. Robot movement
        """
        ## Initialise map
        ### TODO

        ## Detect obstacles to update map
        ### TODO Add ultrasonic detector 
        # Use ultrasonic sensor to update map

        ## Path Planning
        start_node = wayp_all[curr]
        target_node = wayp_all[curr+1]

        G = Graph()
        G.djikstras(start_node, target_node)
        path, dist = G.get_shortest_distance(target_node)
        #print(path, dist)


        ## Robot movement
        # Move to target waypoint

        # Wait 10 seconds at/near waypoint to check
        time.sleep(10)

        
        

        
    

    