"""
 -
Author: Thomas Pardy
Date Modified: 2023-08-23

"""
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/tom/4191-Robot")
from Navigation.graph import Graph, Node
from math import comb
# Recieve position from rotary encoders
# Recieve readouts from ultrasonic sensor

class Map:
    """Generates map o arena, navigates shortest path. Online updating of path
    with detected obstacles factored in"""
    def __init__(self, arena: tuple, radius: float, loc=(0,0,np.pi/2)):
        """
        Initializes variables
        """
        self.arena_dimensions = arena
        self.radius = radius
        self.G = Graph()
        self.location = loc
        self.path = []
        self.object_size = (250,90)
        self.obstacle_corners = []

    def generate_map(self):
        """
        Generates grid-like nodal map of arena.
        """
        # Computes and initializes number of nodes
        row_n = self.arena_dimensions[0] // self.radius - 1
        col_n = self.arena_dimensions[1] // self.radius - 1
        nodes = []
        for i in range(row_n):
            row = []
            for j in range(col_n):
                node = Node(f"({i},{j})")
                node.xy = [i*self.radius + self.radius, j*self.radius + self.radius]

                row.append(node)

            nodes.append(row)


        # Adds neighbours to corresponding nodes
        for i in range(row_n):
            for j in range(col_n):
                if i > 0:
                    nodes[i][j].add_neighbour(nodes[i - 1][j], 1) # Up
                if i < row_n - 1:
                    nodes[i][j].add_neighbour(nodes[i + 1][j], 1)  # Down
                if j > 0:
                    nodes[i][j].add_neighbour(nodes[i][j - 1], 1) # Left
                if j < col_n - 1:
                    nodes[i][j].add_neighbour(nodes[i][j + 1], 1)
        # Adds nodes to graph
        for i in nodes:
            for j in i:
                self.G.add_node(j)

    def update_location(self, pose) -> None:
        """
        Updates predicted location on nodal map
        """
        self.location = pose


    def remap(self, ultrasonic_readout, object_size: tuple) -> None:
        """
        Re calibrates map with object blocked out on nodes.
        """
        x, y, th = self.location
        obs_x = x + object_size[0]/2 * np.cos(th) + 2*ultrasonic_readout * np.cos(th)
        obs_y = y + object_size[1]/2 * np.sin(th) + 2*ultrasonic_readout * np.sin(th)
        self.G.reset_graph()
        closest_node = self.G.get_nearest_node((obs_x, obs_y))
        obstacle_nodes = self.G.adjacent_nodes(closest_node, object_size)
        obstacle_xy = []
        for node in obstacle_nodes:
            obstacle_xy.append(node.xy)
        corners = [(min([x[0] for x in obstacle_xy]) - self.radius, min([x[1] for x in obstacle_xy]) - self.radius),
                    (min([x[0] for x in obstacle_xy]) - self.radius, max([x[1] for x in obstacle_xy]) + self.radius),
                    (max([x[0] for x in obstacle_xy]) + self.radius, min([x[1] for x in obstacle_xy]) - self.radius),
                    (max([x[0] for x in obstacle_xy]) + self.radius, max([x[1] for x in obstacle_xy]) + self.radius),
                    ]

        self.obstacle_corners.append(corners)
        for node in obstacle_nodes:
            self.G.set_obstacle(node)

    def update_path(self, waypoint) -> None:
        """
        Updates path to avoid any new obstacles
        """
        start_node = self.G.get_nearest_node(self.location[:2])
        end_node = self.G.get_nearest_node(waypoint[:2])
        if end_node is not None:
            self.G.djikstras(start_node, end_node)
            _, self.path = self.G.get_shortest_distance(end_node)
        else:
            print("Cant find waypoint.")
        self.shorten_shortest_path()

    def get_path_xy(self) -> list:
        """
        Returns path
        """
        path_xy = []
        for node in self.path:
            path_xy.append(self.G[eval(node)].xy)
        return path_xy

    def check_obstacle(self, ultrasonic_readout: float, detect_distance) -> bool:
        """Checks if ultrasonic readout detected obstacle, if so remaps and updates path"""

        if ultrasonic_readout < detect_distance:
            self.remap(ultrasonic_readout, self.object_size)
            self.update_path
            return True
        else:
            return False
    def ccw(self, A, B, C):
        """ccw: Returns true if points are in counter clockwise order"""
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    def line_intersect(self, A, B, C, D):
        """line_intersect: Returns true if line segments AB and CD intersect"""
        return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)
    def line_obstacle_free(self, A, B):
        """
        line_obstacle_free: Returns true if line segment AB is obstacle free
        """
        for corner in self.obstacle_corners:
            if self.line_intersect(A.xy, B.xy, corner[0], corner[1]) or self.line_intersect(A.xy, B.xy, corner[0], corner[2]) or self.line_intersect(A.xy, B.xy, corner[1], corner[3]) or self.line_intersect(A.xy, B.xy, corner[2], corner[3]):
                return False
        return True

    def shorten_shortest_path(self) -> None:
        """
        Shortens path by removing nodes that are not needed.
        """
        start_node = self.G[eval(self.path[0])]
        i = 1
        while i < len(self.path) - 1:
            current_node = self.G[eval(self.path[i])]
            
            # Check if line between start and current node is obstacle free
            if self.line_obstacle_free(start_node, current_node):
                self.path.pop(i)

            else:
                start_node = current_node
                i += 1
                
            
    def bezier_curve(self, t):
        """
        Returns bezier curve of path
        """
        path_xy = self.get_path_xy()
        n = len(path_xy)
        # Basis
        basis_vals = []
        for i in range(n):
            basis_vals.append(comb(n-1,i) * ((1-t)**(n-1-i) * (t**i)))
        
        x = 0
        y = 0
        for i in range(n):
            x += basis_vals[i] * path_xy[i][0]
            y += basis_vals[i] * path_xy[i][1]

        return (x,y)
    def draw_arena(self, draw_path=True) -> None:
        """draw_arena: Draws arena as graph"""
        G_img = nx.Graph()
        # Draw Nodes
        for node_name in self.G.nodes:
            node = self.G[eval(node_name)]
            G_img.add_node(node.name, pos=node.xy)

        for node_name in self.G.nodes:
            node = self.G[eval(node_name)]
            for edge in node.neighbours:
                G_img.add_edge(node.name, edge[0].name)

        node_idx = 0
        while node_idx < len(self.path) - 1:
            G_img.add_edge(self.path[node_idx], self.path[node_idx + 1])
            node_idx += 1


        # Draw Bezier
        # t = np.linspace(0, 1, 100)
        # x_vals = []
        # y_vals = []
        # for i in t:
        #     x, y = self.bezier_curve(i)
        #     x_vals.append(x)
        #     y_vals.append(y)

        # plt.plot(x_vals, y_vals, 'r--')
        # Draw boundary
        max_x = self.arena_dimensions[0]
        max_y = self.arena_dimensions[1]
        G_img.add_node("A", pos=(0, 0))
        G_img.add_node("B", pos=(max_x, 0))
        G_img.add_node("C", pos=(max_x, max_y))
        G_img.add_node("D", pos=(0, max_y))

        G_img.add_edges_from([("A", "B"), ("B", "C"), ("C", "D"), ("D", "A")])
        # Drawing Properties
        node_colors = []
        for node in G_img.nodes:
            if node in ["A","B","C","D"]:
                node_colors.append('black')

            elif self.G[eval(node)].xy == self.location[:2]:
                node_colors.append('yellow')

            elif self.G[eval(node)].is_obstacle:
                node_colors.append('green')

            elif node in self.path:
                node_colors.append('red')
            else:
                node_colors.append('skyblue')
        node_sizes = [20 if not node in ["A","B","C","D"] else 10 for node in G_img.nodes]
        node_positions = {node: data["pos"] for node, data in G_img.nodes(data=True)}

        if draw_path:
            edge_colors = []
            edge_width = []
            for edge in G_img.edges:

                if edge[0] in ["A", "B", "C", "D"] or edge[1] in ["A", "B", "C", "D"]:
                    edge_colors.append("black")
                    edge_width.append(1)
                elif edge[0] in self.path and edge[1] in self.path:
                    edge_colors.append("red")
                    edge_width.append(1)


                else:
                    edge_colors.append("black")
                    edge_width.append(1)
        else:
            edge_colors = ["black" for _ in G_img.edges]
            edge_width = [1 for _ in G_img.edges]


        nx.draw(G_img, pos=node_positions, node_size=node_sizes, with_labels=False, node_color=node_colors, edge_color=edge_colors, width=edge_width)

        plt.show()
        


if __name__ == '__main__':
    map_test = Map((1000, 1000), 50, loc=(500,50,np.pi/2))
    map_test.generate_map()
    end_node_xy = (1000, 1000)
    end_node = map_test.G.get_nearest_node(end_node_xy)
    map_test.update_path(end_node_xy)
    path = map_test.get_path_xy()
    map_test.draw_arena()
    map_test.remap(100,(350, 200))
    map_test.update_path(end_node_xy)
    map_test.shorten_shortest_path()
    map_test.remap(300,(300, 100))
    map_test.draw_arena(draw_path=True)
