from eyepy import *
from quadtree import Region, find_valid_edges, quadtree, read_p1

FILENAME = "./corner.pbm"
VACANT_COL = GREEN
OCCUPIED_COL = RED
EDGE_COL = WHITE

image = read_p1(FILENAME)
regions = quadtree(image)

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

nodes = { node: region.get_centre() for node, region in enumerate(regions.vacant) }
print(nodes)

edges = find_valid_edges(regions)
print(edges)

for src, dst_set in edges.items():
    for dst in dst_set:
        if src > dst:
            # backwards edge don't print
            continue
        
        LCDLine(nodes[src], nodes[dst], EDGE_COL)
