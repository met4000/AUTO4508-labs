from eyepy import *

from astar import path_from_file


reachable, path, distance, _ = path_from_file("./branch.txt")
print("Reachable:", reachable)
print("Path:", path)
print("Path Length:", distance)
