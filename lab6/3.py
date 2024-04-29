from __future__ import annotations

from eye import SIMSetRobot
from eyepy import *

import sys
sys.path.append("../lab3")
sys.path.append("../lab4")
sys.path.append("../lab5")

from astar import astar_path, make_distance_matrix, make_heuristic_matrix
from brushfire import brushfire, closest_node_to_point, voronoi_graph
from distbug import distbug_abs
from quadtree import read_p1

FILENAME = "./u.pbm"

MAP_SCALING = 31 # scaling factor from image to world
LCD_SCALING = 2

PRINT = True
FREE_COLOUR = WHITE
POINT_COLOUR = BLACK

SRC_COL = GRAY
DST_COL = WHITE
PATH_COL = WHITE
PATH_NEXT_COL = CYAN

start_pos_padding = Vector(120, -120)

LCDSetPointMap(lambda p: (p * LCD_SCALING).round())


image = read_p1(FILENAME)
voronoi_points = brushfire(image, print=PRINT, free_colour=FREE_COLOUR, point_colour=POINT_COLOUR, lcd_scaling=LCD_SCALING)

image_coord_segment = (
    (-1, -1),
    (image.resolution.WIDTH, image.resolution.HEIGHT)
)
world_coord_segment = (
    Point(0, image.resolution.HEIGHT) * MAP_SCALING,
    Point(image.resolution.WIDTH, 0) * MAP_SCALING
)

map_nodes, edges = voronoi_graph(voronoi_points)

point_map = make_coord_map(image_coord_segment, world_coord_segment)
inv_point_map = make_coord_map(world_coord_segment, image_coord_segment)
nodes = { node: point_map(Point(*pos)) for node, pos in map_nodes.items() }

src = closest_node_to_point(map_nodes, inv_point_map(point_map(Point(0, 0)) + start_pos_padding))
dst = closest_node_to_point(map_nodes, (image.resolution.WIDTH - 1, image.resolution.HEIGHT - 1))

distance_matrix = make_distance_matrix(nodes, edges)
heuristic_matrix = make_heuristic_matrix(nodes, dst)

reachable, path, distance = astar_path(distance_matrix=distance_matrix, heuristic_matrix=heuristic_matrix, src=src, dst=dst)
src_pos = Point(*nodes[src])
dst_pos = Point(*nodes[dst])

print("Reachable:", reachable)
if not reachable:
    exit()

# print("Path:", path)
# print("Path Positions:", [nodes[node] for node in path])
print("Path Length:", distance)

# print path to lcd

LCDSetPointMap(lambda p: (inv_point_map(p) * LCD_SCALING).round())

for i in range(1, len(path)):
    src = path[i - 1]
    dst = path[i]
    LCDLine(nodes[src], nodes[dst], PATH_COL)

LCDCircle(src_pos, 8, SRC_COL)
LCDCircle(dst_pos, 8, DST_COL)

start_pos = src_pos.round()
VWStop()
SIMSetRobot(0, start_pos.x, start_pos.y, 4, 0)
VWSetPosition(src_pos.round(), 0)

for prev_node_index, next_node in enumerate(path[1:]):
    next_pos = nodes[next_node]
    LCDLine(nodes[path[prev_node_index]], next_pos, PATH_NEXT_COL)
    distbug_abs(next_pos, hit_distance=20, lin_speed=170, end_threshold=12, lcd_print=False)
