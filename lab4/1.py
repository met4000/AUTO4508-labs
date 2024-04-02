import re

from eyepy import *


def make_distance_matrix(file_path: str) -> list[list[float]]:
    # load in edge list (and node coords)

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
    
    # convert to (distance) matrix

    n_nodes = len(nodes)
    distance_matrix: list[list[float]] = []

    for src in range(n_nodes):
        src_pos = nodes[src]

        distance_matrix.append([])
        for dst in range(n_nodes):
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

if __name__ == "__main__":
    matrix = make_distance_matrix("./nodes.txt")
    print(matrix)
