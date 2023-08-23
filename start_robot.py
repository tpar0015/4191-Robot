# Call modules for robot
import time
from Navigation.graph import Node, Graph
from ultrasonic import Ultrasonic




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

    res = ast.literal_eval(ini_list)

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

    # Initialise variables to start robot
    complete = False
    curr_wayp = wayp_0 
    next_wayp = wayp_1

    # Move from waypoint 0 to waypoint 1
    while not complete:
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
        start_node = curr_wayp
        target_node = next_wayp

        G = Graph()
        G.djikstras(start_node, target_node)
        path, dist = G.get_shortest_distance(target_node)
        #print(path, dist)


        ## Robot movement
        # Move to target waypoint
        


        if curr_wayp == wayp_2:
            complete = True
    

    