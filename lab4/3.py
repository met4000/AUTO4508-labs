from eyepy import *
from eye import SIMSetRobot

import sys
sys.path.append("../lab3")

from astar import path_from_file
from distbug import distbug_abs


reachable, path, distance, nodes = path_from_file("./branch.txt")
src_pos = nodes[0]

print("Reachable:", reachable)
if not reachable:
    exit()

print("Path:", path)
print("Path Positions:", [nodes[node] for node in path])
print("Path Length:", distance)

padding = Vector(100, 100)
start_pos = (src_pos + padding).round()
VWStop()
SIMSetRobot(0, start_pos.x, start_pos.y, 4, 0)
VWSetPosition(src_pos.round(), 0)

for next_node in path[1:]:
    next_pos = nodes[next_node]
    distbug_abs(next_pos, hit_distance=100, lin_speed=170, end_threshold=20)
