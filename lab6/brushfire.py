from __future__ import annotations
from collections import deque
import random

from eyepy import *

import sys
sys.path.append("../lab5")

def occupied(image: Image, p: IntPointLike, *, default: bool = False) -> bool:
    if not Image.point_in_image(image.resolution, p):
        return default
    return image.get_gray(p) == 1

def brushfire(image: Image, *, print: bool = False, free_colour: Colour = WHITE, point_colour: Colour = BLACK, lcd_scaling: float = 1) -> set[IntPoint]:
    def printPixelArea(p: PointLike, col: Colour):
        p = Point(*p)
        LCDArea(p, p + Point(1, 1) * (lcd_scaling - 1), col)

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

    if print:
        for x in range(image.resolution.WIDTH):
            for y in range(image.resolution.HEIGHT):
                p = IntPoint(x, y)

                # col: Colour
                if p in pixel_col:
                    col_id = pixel_col[p]
                    col = col_map[col_id]
                else:
                    # vacant
                    col = WHITE
                
                printPixelArea(p, col)

    voronoi_points: set[IntPoint] = set()
    pixel_layer: dict[IntPoint, int] = { p: 1 for p in pixel_col.keys() }

    layer = 2
    modified = True
    while modified:
        modified = False

        for y in range(image.resolution.HEIGHT):
            for x in range(image.resolution.WIDTH):
                p = IntPoint(x, y)

                if p in voronoi_points: continue
                if p in pixel_col: continue

                double_col: bool = False
                col: int | None = None
                for dx in range(-1, 1 + 1):
                    for dy in range(-1, 1 + 1):
                        rel_p = IntPoint(p.x + dx, p.y + dy)
                        if rel_p == p: continue

                        if rel_p not in pixel_col: continue

                        # has a colour

                        # ignore if not previous layer
                        if pixel_layer[rel_p] != layer - 1: continue
                        
                        new_col = pixel_col[rel_p]

                        if col is not None and col != new_col:
                            double_col = True
                            break

                        col = new_col
                    
                    if double_col:
                        break

                if col is None:
                    # nothing yet
                    continue

                modified = True
                pixel_layer[p] = layer
                
                if double_col:
                    # Voronoi point
                    voronoi_points.add(p)
                    if print: printPixelArea(p, point_colour)
                    continue

                # else, single colour
                pixel_col[p] = col

                up_p = IntPoint(p.x, p.y - 1)
                left_p = IntPoint(p.x - 1, p.y)
                if (
                    up_p in pixel_col and pixel_col[up_p] != col and pixel_layer[up_p] == layer
                ) or (
                    left_p in pixel_col and pixel_col[left_p] != col and pixel_layer[left_p] == layer
                ):
                    # 2-wide equidistant section - also a Voronoi point
                    voronoi_points.add(p)
                    if print: printPixelArea(p, point_colour)
                    continue

                # else, continue
                if print: printPixelArea(p, col_map[col])

        layer += 1

    if not print:
        for p in voronoi_points:
            printPixelArea(p, point_colour)
    
    return voronoi_points

def voronoi_graph(voronoi_points: set[IntPoint]) -> tuple[dict[int, Point], dict[int, set[int]]]:
    nodes: dict[int, IntPoint] = {}
    inv_nodes: dict[IntPoint, int] = {}
    for node, p in enumerate(voronoi_points):
        nodes[node] = p
        inv_nodes[p] = node

    edges: dict[int, set[int]] = { node: set() for node in nodes.keys() }
    
    for node, p in nodes.items():
        for dx in range(-1, 1 + 1):
            for dy in range(-1, 1 + 1):
                rel_p = IntPoint(p.x + dx, p.y + dy)
                if rel_p not in voronoi_points: continue

                rel_node = inv_nodes[rel_p]
                edges[node].add(rel_node)
    
    return { node: Point(*p) for node, p in nodes.items() }, edges

def closest_node_to_point(nodes: dict[int, Point], p: PointLike) -> int:
    p = Point(*p)

    min_node = -1
    min_dst = math.inf

    for node, node_pos in nodes.items():
        dst = abs(p - node_pos)
        if dst < min_dst:
            min_node = node
            min_dst = dst
    
    return min_node
