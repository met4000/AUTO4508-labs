from eye import SIMSetRobot
from eyepy import *

import sys
sys.path.append("../lab3")
sys.path.append("../lab4")

from astar import astar_path, make_distance_matrix, make_heuristic_matrix
from distbug import distbug_abs
from quadtree import Region, closest_region_to_point, find_valid_edges, quadtree, read_p1

FILENAME = "./corner.pbm"
MAP_SCALING = 31 # scaling factor from image to world

PADDING_ITERATIONS = 5

VACANT_COL = GREEN
OCCUPIED_COL = RED
EDGE_COL = WHITE

SRC_COL = CYAN
DST_COL = BLUE
PATH_COL = BLUE
PATH_NEXT_COL = CYAN

start_pos_padding = Vector(0, 0)

raw_image = read_p1(FILENAME)

# image processing to pad obstacles

dilation_border_value = 1
def dilation(p: IntPoint, get_or_default: Callable[[IntPointLike, float], float]) -> int:
    for offset in itertools.product(*((range(-1, 1+1),) * 2)):
        if get_or_default((Point(*p) + cast(tuple[int, int], offset)).round(), dilation_border_value) > 0:
            return 1
    return 0

padded_image = raw_image
for _ in range(PADDING_ITERATIONS):
    padded_image = padded_image.apply_local_op(dilation)

regions = quadtree(padded_image)

def print_region_bounds(region: Region, col: Colour):
    LCDLine(region.p1, (region.p2.x, region.p1.y), col) # top
    LCDLine(region.p1, (region.p1.x, region.p2.y), col) # left
    LCDLine((region.p2.x, region.p1.y), region.p2, col) # right
    LCDLine((region.p1.x, region.p2.y), region.p2, col) # bottom

for region in regions.vacant:
    print_region_bounds(region, VACANT_COL)
    LCDPixel(region.get_centre(), col=VACANT_COL)

for region in regions.occupied:
    print_region_bounds(region, OCCUPIED_COL)
    LCDPixel(region.get_centre(), col=OCCUPIED_COL)

map_nodes = { node: region.get_centre() for node, region in enumerate(regions.vacant) }
# print("Map node coords:")
# print(map_nodes)

edges = find_valid_edges(regions)
# print("Edges:")
# print(edges)

for src, dst_set in edges.items():
    for dst in dst_set:
        if src > dst:
            # backwards edge don't print
            continue
        
        LCDLine(map_nodes[src], map_nodes[dst], EDGE_COL)


# convert coords to world coords (top left 0, 0 => bottom left 0, 0)

image_coord_segment = (
    (-1, -1),
    (padded_image.resolution.WIDTH, padded_image.resolution.HEIGHT)
)
world_coord_segment = (
    Point(0, padded_image.resolution.HEIGHT) * MAP_SCALING,
    Point(padded_image.resolution.WIDTH, 0) * MAP_SCALING
)

point_map = make_coord_map(image_coord_segment, world_coord_segment)
nodes = { node: point_map(Point(*pos)) for node, pos in map_nodes.items() }

# compute path

src = closest_region_to_point(regions.vacant, (0, 0))
dst = closest_region_to_point(regions.vacant, (padded_image.resolution.WIDTH - 1, padded_image.resolution.HEIGHT - 1))

distance_matrix = make_distance_matrix(nodes, edges)
heuristic_matrix = make_heuristic_matrix(nodes, dst)

reachable, path, distance = astar_path(distance_matrix=distance_matrix, heuristic_matrix=heuristic_matrix, src=src, dst=dst)
src_pos = Point(*nodes[src])
dst_pos = Point(*nodes[dst])

print("Reachable:", reachable)
if not reachable:
    exit()

print("Path:", path)
print("Path Positions:", [nodes[node] for node in path])
print("Path Length:", distance)

# print path to lcd

inv_point_map = make_coord_map(world_coord_segment, image_coord_segment)
LCDSetPointMap(lambda p: inv_point_map(p).round())

for i in range(1, len(path)):
    src = path[i - 1]
    dst = path[i]
    LCDLine(nodes[src], nodes[dst], PATH_COL)

LCDCircle(src_pos, 8, SRC_COL)
LCDCircle(dst_pos, 8, DST_COL)

start_pos = (src_pos + start_pos_padding).round()
VWStop()
SIMSetRobot(0, start_pos.x, start_pos.y, 4, 0)
VWSetPosition(src_pos.round(), 0)

for prev_node_index, next_node in enumerate(path[1:]):
    next_pos = nodes[next_node]
    LCDLine(nodes[path[prev_node_index]], next_pos, PATH_NEXT_COL)
    distbug_abs(next_pos, hit_distance=100, lin_speed=170, end_threshold=20, lcd_print=False)
