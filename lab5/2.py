from eyepy import *
from quadtree import find_valid_edges, quadtree, read_p1

FILENAME = "./blocks.pbm"

image = read_p1(FILENAME)
regions = quadtree(image)

nodes = { node: region.get_centre() for node, region in enumerate(regions.vacant) }
print(nodes)

edges = find_valid_edges(regions)
print(edges)

input()
