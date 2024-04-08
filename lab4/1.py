from eyepy import *

from astar import graph_from_file, make_distance_matrix


nodes, edges = graph_from_file("./branch.txt")
matrix = make_distance_matrix(nodes, edges)
print(matrix)
