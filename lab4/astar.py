from __future__ import annotations
from queue import PriorityQueue
import re

from eyepy import *


def graph_from_file(file_path: str) -> tuple[dict[int, Point], dict[int, set[int]]]:
    """returns coords and edges"""
    
    nodes: dict[int, Point] = {}
    edges: dict[int, set[int]] = {}

    with open(file_path) as file:
        for i, line in enumerate(file):
            stripped_line = line.strip()
            node_x_str, node_y_str, *node_connection_strs = re.split(r"\s+", stripped_line)

            node_x = int(node_x_str)
            node_y = int(node_y_str)
            nodes[i] = Point(x=node_x, y=node_y)

            edges[i] = set()
            for dst_node_str in node_connection_strs:
                dst_node = int(dst_node_str) - 1 # files are 1-indexed, matrix will be 0-indexed
                edges[i].add(dst_node)
    
    return nodes, edges

def make_distance_matrix(nodes: dict[int, Point], edges: dict[int, set[int]]) -> list[list[float]]:
    distance_matrix: list[list[float]] = []

    for src in range(len(nodes)):
        src_pos = nodes[src]

        distance_matrix.append([])
        for dst in range(len(nodes)):
            distance: float | None = None

            if dst == src:
                distance = 0
            elif dst not in edges[src]:
                distance = -1
            else:
                dst_pos = nodes[dst]
                distance = abs(dst_pos - src_pos)
            
            distance_matrix[src].append(distance)

    return distance_matrix

def make_heuristic_matrix(nodes: dict[int, Point], dst: int) -> list[float]:
    heuristic_matrix: list[float] = []

    dst_pos = nodes[dst]
    for node in range(len(nodes)):
        heuristic: float | None = None

        if node == dst:
            heuristic = 0
        else:
            node_pos = nodes[node]
            heuristic = abs(dst_pos - node_pos)
        
        heuristic_matrix.append(heuristic)

    return heuristic_matrix

def astar_path(*, distance_matrix: list[list[float]], heuristic_matrix: list[float], src: int, dst: int) -> tuple[bool, list[int], float]:
    """returns reachable, path, and path length"""

    queue: PriorityQueue[tuple[float, list[int]]] = PriorityQueue()

    for node, node_distance in enumerate(distance_matrix[src]):
        if node_distance <= 0.0:
            # self-edge or no edge
            continue

        node_heuristic = heuristic_matrix[node]
        weight = node_distance + node_heuristic
        path = [src, node]
        queue.put((weight, path))
    
    while queue.qsize() > 0:
        weight, path = queue.get()
        current_node = path[-1]

        if current_node == dst:
            # reached the end

            # weight is the same as the distance
            # since the heuristic for the
            # destination is 0
            return True, path, weight
        
        current_path_length = weight - heuristic_matrix[current_node]
        for next_node, next_node_distance in enumerate(distance_matrix[current_node]):
            if next_node_distance <= 0.0:
                # self-edge or no edge
                continue

            if next_node in path:
                # self-intersecting
                continue

            next_node_heuristic = heuristic_matrix[next_node]
            new_weight = current_path_length + next_node_distance + next_node_heuristic
            queue.put((new_weight, path + [next_node]))
    
    return False, [], math.inf

def path_from_file(file_path: str, *, src: Optional[int] = None, dst: Optional[int] = None) -> tuple[bool, list[int], float, dict[int, Point]]:
    """defaults to first node is src, and last node is dst"""
    
    nodes, edges = graph_from_file(file_path)

    src = 0 if src is None else src
    dst = len(nodes) - 1 if dst is None else dst

    distance_matrix = make_distance_matrix(nodes, edges)
    heuristic_matrix = make_heuristic_matrix(nodes, dst)

    reachable, path, distance = astar_path(distance_matrix=distance_matrix, heuristic_matrix=heuristic_matrix, src=src, dst=dst)
    return reachable, path, distance, nodes
