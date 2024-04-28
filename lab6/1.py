from collections import deque
import random
from eyepy import *

import sys
sys.path.append("../lab5")

from quadtree import read_p1

FILENAME = "./u.pbm"


def occupied(image: Image, p: IntPointLike, *, default: bool = False) -> bool:
    if not Image.point_in_image(image.resolution, p):
        return default
    return image.get_gray(p) == 1


image = read_p1(FILENAME)
LCDImageBinary(image) # ! TEMP

# pixel to col id
pixel_col: dict[IntPoint, int] = {}

# add out of bounds area (implicit walls) to colouring map
# top (1) and bottom (3)
for i, y in enumerate([-1, image.resolution.HEIGHT]):
    for x in range(image.resolution.WIDTH):
        pixel_col[IntPoint(x, y)] = 1 + 2 * i
# left (2) and right (0)
for y in range(image.resolution.HEIGHT):
    for i, x in enumerate([-1, image.resolution.WIDTH]):
        pixel_col[IntPoint(x, y)] = 2 - 2 * i
# corners are implicitly nothing


# scan to find objects

object_count: int = 4 # starts as the 4 walls
seen: set[IntPoint] = set()

for y in range(image.resolution.HEIGHT):
    for x in range(image.resolution.WIDTH):
        p = IntPoint(x, y)
        if p in seen: continue

        if not occupied(image, p): continue
        
        # new object found
        # DFS to find all other points
        col = object_count
        object_count += 1
        seen.add(p)

        search_stack: deque[IntPoint] = deque((p,))

        while search_stack:
            current_p = search_stack.pop()
            pixel_col[current_p] = col

            for dx in range(-1, 1 + 1):
                for dy in range(-1, 1 + 1):
                    new_p = IntPoint(current_p.x + dx, current_p.y + dy)
                    if new_p in seen: continue
                    seen.add(new_p)

                    if not occupied(image, new_p): continue
                    
                    search_stack.append(new_p)


# print with object colour

cols = [IPPRGB2Col(IPPHSI2RGB((round(360 / object_count * i), 255, 255))) for i in range(object_count)]
random.shuffle(cols)
col_map: dict[int, Colour] = dict(enumerate(cols))

for x in range(image.resolution.WIDTH):
    for y in range(image.resolution.HEIGHT):
        p = IntPoint(x, y)

        col: Colour
        if p in pixel_col:
            col_id = pixel_col[p]
            col = col_map[col_id]
        else:
            # vacant
            col = WHITE
        
        LCDPixel(p, col)

input()
