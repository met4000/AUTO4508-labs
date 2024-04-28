from __future__ import annotations

from eyepy import *

import sys
sys.path.append("../lab5")

from brushfire import brushfire
from quadtree import read_p1

FILENAME = "./u.pbm"

PRINT = True
FREE_COLOUR = WHITE
POINT_COLOUR = BLACK


image = read_p1(FILENAME)
voronoi_points = brushfire(image, print=PRINT, free_colour=FREE_COLOUR, point_colour=POINT_COLOUR)

input()
