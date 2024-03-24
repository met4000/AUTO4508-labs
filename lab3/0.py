from eyepy import *

distances = LIDARGet()

_, display_max_y = LCDGetSize()
def transform_point(p: Point) -> IntPoint:
    transformed_point = ((1, 0), (0, -1)) @ p.as_vector()
    screen_point = Point(0, display_max_y - 1) + transformed_point
    return screen_point.round()

graph_root = Point(0, 10)

distances = LIDARGet()
for i, distance in enumerate(distances):
    point_root = graph_root + (i, 0)
    LCDLine(transform_point(point_root), transform_point(point_root + (0, distance / 20)), WHITE)

input()
