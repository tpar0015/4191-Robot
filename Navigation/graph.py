"""
Author: Thomas Pardy
Date Modified: <2023-08-21 Mon>
Module containing graph related classes for mapping of arena.
"""

import heapq


class Node:
    def __init__(self, name):
        self.name = name
        self.visited = False
        self.distance = float('inf')
        self.neighbours = []
        self.prev_node = None
        self.xy = [float('inf'), float('inf')]


    def add_neighbour(self, neighbour, weight):
        self.neighbours.append((neighbour, weight))


    def __lt__(self, other_node):
        return self.distance < other_node.distance





class Graph:

    def __init__(self):
        self.nodes = {}


    def add_node(self, node: Node):
        self.nodes[node.name] = node


    def djikstras(self, start_node, target_node):
        start_node.distance = 0
        heap = [(0, start_node)]


        while heap:
            current_distance, current_node = heapq.heappop(heap)

            if current_node == target_node:
                break

            if current_node.visited:
                continue

            current_node.visited = True

            for neighbour,  weight in current_node.neighbours:
                if not neighbour.visited:
                    new_distance = current_distance + weight
                    if new_distance < neighbour.distance:
                        neighbour.distance = new_distance
                        neighbour.prev_node = current_node
                        heapq.heappush(heap, (new_distance, neighbour))



    def get_shortest_distance(self, target: Node):
        path = []
        current_node = target

        while current_node.prev_node is not None:
            path.insert(0,current_node.name)
            current_node = current_node.prev_node
        path.insert(0,current_node.name)
        return target.distance, path



if __name__ == '__main__':
    nodes = [[Node(f"({row},{col})") for col in range(3)] for row in range(3)]

    for row in range(3):
        for col in range(3):
            if row > 0:
                nodes[row][col].add_neighbour(nodes[row - 1][col], 1) # Up

            if row < 2:
                nodes[row][col].add_neighbour(nodes[row + 1][col], 1) # Down

            if col > 0:
                nodes[row][col].add_neighbour(nodes[row][col - 1], 1) # Left

            if col < 2:
                nodes[row][col].add_neighbour(nodes[row][col + 1], 1) # Right


    G = Graph()
    for row in nodes:
        for node in row:
            G.add_node(node)

    start_node = nodes[1][1]
    target_node = nodes[2][2]
    G.djikstras(start_node, target_node)
    path, dist = G.get_shortest_distance(target_node)
    print(path, dist)
