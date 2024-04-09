from eyepy import *
from eye import SIMSetRobot

import sys
sys.path.append("../lab3")

from astar import astar_path, graph_from_file, make_distance_matrix, make_heuristic_matrix
from distbug import distbug_abs


GRAPH_FILE = "./branch.txt"

nodes, edges = graph_from_file(GRAPH_FILE)

path_src = 0
path_dst = len(nodes) - 1

distance_matrix = make_distance_matrix(nodes, edges)
heuristic_matrix = make_heuristic_matrix(nodes, path_dst)

reachable, path, distance = astar_path(distance_matrix=distance_matrix, heuristic_matrix=heuristic_matrix, src=path_src, dst=path_dst)
src_pos = nodes[0]

print("Reachable:", reachable)
if not reachable:
    exit()

print("Path:", path)
print("Path Positions:", [nodes[node] for node in path])
print("Path Length:", distance)


# print graph and path

LCD_NODE_SIZE = 8
LCD_NODE_COLOUR = WHITE
LCD_LINE_COLOUR = WHITE

LCD_SRC_NODE_COLOUR = YELLOW
LCD_DST_NODE_COLOUR = GREEN
LCD_PATH_COLOUR = GREEN

LCDSetPointMap(lcd_default_world_coord_map)

# edges
for src, dst_list in edges.items():
    for dst in dst_list:
        LCDLine(nodes[src], nodes[dst], LCD_LINE_COLOUR)

# path
for i in range(1, len(path)):
    src = path[i - 1]
    dst = path[i]
    LCDLine(nodes[src], nodes[dst], LCD_PATH_COLOUR)

# nodes
for node_id, node_pos in nodes.items():
    col: Colour = LCD_NODE_COLOUR
    if node_id == path_src:
        col = LCD_SRC_NODE_COLOUR
    elif node_id == path_dst:
        col = LCD_DST_NODE_COLOUR
    
    LCDCircle(node_pos, LCD_NODE_SIZE, col)


padding = Vector(100, 100)
start_pos = (src_pos + padding).round()
VWStop()
SIMSetRobot(0, start_pos.x, start_pos.y, 4, 0)
VWSetPosition(src_pos.round(), 0)

for next_node in path[1:]:
    next_pos = nodes[next_node]
    distbug_abs(next_pos, hit_distance=100, lin_speed=170, end_threshold=20, lcd_print=False)
