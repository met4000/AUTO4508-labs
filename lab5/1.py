from eyepy import *
from quadtree import Region, quadtree, read_p1

FILENAME = "./blocks.pbm"
VACANT_COL = GREEN
OCCUPIED_COL = RED

image = read_p1(FILENAME)
vacant, occupied = quadtree(image)

def print_region_bounds(region: Region, col: Colour):
    LCDLine(region.p1, (region.p2.x, region.p1.y), col) # top
    LCDLine(region.p1, (region.p1.x, region.p2.y), col) # left
    LCDLine((region.p2.x, region.p1.y), region.p2, col) # right
    LCDLine((region.p1.x, region.p2.y), region.p2, col) # bottom

for region in vacant:
    print_region_bounds(region, VACANT_COL)
    LCDPixel(region.get_centre(), col=VACANT_COL)

for region in occupied:
    print_region_bounds(region, OCCUPIED_COL)
    LCDPixel(region.get_centre(), col=OCCUPIED_COL)

print(f"Vacant Centres ({len(vacant)}):")
for region in vacant:
    print(region.get_centre())

input()
