from __future__ import annotations
from eyepy import *

def read_p1(filename: str) -> Image:
    with open(filename) as file:
        magicNumber = file.read(2)
        if magicNumber != "P1":
            raise ValueError(f"expected magic number of '{filename}' to be 'P1': found '{magicNumber}'")

        def read_next() -> str:
            char = file.read(1)
            while char.isspace():
                char = file.read(1)
            
            chars = char
            char = file.read(1)
            while not char.isspace():
                chars += char
                char = file.read(1)

            return chars

        width_str = read_next()
        width = int(width_str)

        height_str = read_next()
        height = int(height_str)

        image_data: list[int] = []
        while True:
            char = file.read(1)
            if char == "": break
            if char.isspace(): continue
            if char == "#":
                while char != "\n":
                    char = file.read(1)
            
            if char not in ["0", "1"]:
                raise Exception(f"unexpected '{char}' at position {file.tell()}")

            image_data.append(int(char))

    if len(image_data) != width * height:
        raise Exception(f"got image data of length {len(image_data)}, but expected length {width * height} ({width} * {height})")

    image = Image.from_list(image_data, gray=True, resolution=ImageResolution(width, height))
    return image

class Region(NamedTuple):
    p1: IntPoint
    """Top Left (inclusive)"""

    p2: IntPoint
    """Bottom Right (inclusive)"""

    def get_centre(self) -> IntPoint:
        """Top-Left-most centre point"""
        centre = (Point(*self.p1) + Point(*self.p2)) / 2
        return centre.floor()

class QuadtreeOutput(NamedTuple):
    vacant: list[Region]
    occupied: list[Region]

    def __iadd__(self, regions: QuadtreeOutput):
        self.vacant.extend(regions.vacant)
        self.occupied.extend(regions.occupied)
        return self


def quadtree(image: Image, *, min_size: float = 0) -> QuadtreeOutput:
    return _quadtree_inner(image, Region(IntPoint(0, 0), IntPoint(image.resolution.WIDTH - 1, image.resolution.HEIGHT - 1)), min_size=min_size)

def _quadtree_inner(image: Image, region: Region, min_size: float = 0) -> QuadtreeOutput:
    # check region is valid
    if region.p1.x > region.p2.x or region.p1.y > region.p2.y:
        return QuadtreeOutput([], [])
    
    # check if region is 1 pixel
    if region.p1 == region.p2:
        pixel = image.get_gray(region.p1)
        if pixel == 0:
            # free
            return QuadtreeOutput([region], [])
        elif pixel == 1:
            # occupied
            return QuadtreeOutput([], [region])
        else:
            raise Exception(f"unexpected pixel value '{pixel}'")
    
    if abs(Point(*region.p1) - Point(*region.p2)) < min_size:
        return QuadtreeOutput([], [region])

    # assumption that will be updated and correct after completing the grid search
    all_free = True
    all_occupied = True
    for coords in itertools.product(range(region.p1.x, region.p2.x + 1), range(region.p1.y, region.p2.y + 1)):
        pixel = image.get_gray(coords)
        if pixel == 0:
            # free
            all_occupied = False
        
        elif pixel == 1:
            # occupied
            all_free = False
        
        else:
            raise Exception(f"unexpected pixel value '{pixel}'")
        
        # check if mixed
        if not all_free and not all_occupied:
            break
    
    if all_free and all_occupied:
        raise Exception("region had no contents")
    
    if all_free:
        return QuadtreeOutput([region], [])
    
    if all_occupied:
        return QuadtreeOutput([], [region])
    

    # mixed: recurse

    centre = region.get_centre()

    regions = QuadtreeOutput([], [])
    regions += _quadtree_inner(image, Region(IntPoint(centre.x + 1, region.p1.y), IntPoint(region.p2.x, centre.y))) # top right
    regions += _quadtree_inner(image, Region(region.p1, centre)) # top left
    regions += _quadtree_inner(image, Region(IntPoint(region.p1.x, centre.y + 1), IntPoint(centre.x, region.p2.y))) # bottom left
    regions += _quadtree_inner(image, Region(IntPoint(centre.x + 1, centre.y + 1), region.p2)) # bottom right

    return regions

def interval_intersection(i1: tuple[int, int], i2: tuple[int, int]) -> bool:
    if i1[1] < i1[0]:
        i1 = (i1[1], i1[0])

    if i2[1] < i2[0]:
        i2 = (i2[1], i2[0])

    return i1[0] <= i2[1] and i2[0] <= i1[1]

def find_valid_edges(regions: QuadtreeOutput, *, buffer: int = 0) -> dict[int, set[int]]:
    """finds all possible (undirected) edges with no collisions for the given regions"""
    edges: dict[int, set[int]] = { src: set() for src, _ in enumerate(regions.vacant)}

    for src in range(len(regions.vacant)):
        src_centre = regions.vacant[src].get_centre()
        for dst in range(src + 1, len(regions.vacant)):
            dst_centre = regions.vacant[dst].get_centre()

            for region in regions.occupied:
                # check if line segment intersects region

                # segment on one side => no intersection
                if (max(src_centre.x, dst_centre.x) < region.p1.x - buffer # left
                 or min(src_centre.x, dst_centre.x) > region.p2.x + buffer # right
                 or max(src_centre.y, dst_centre.y) < region.p1.y - buffer # top
                 or min(src_centre.y, dst_centre.y) > region.p2.y + buffer # bottom
                ): continue
                
                # overlap in projections onto axis => intersection
                if (interval_intersection((src_centre.x, dst_centre.x), (region.p1.x - buffer, region.p2.x + buffer))
                 and interval_intersection((src_centre.y, dst_centre.y), (region.p1.y - buffer, region.p2.y + buffer))
                ): break
            else:
                # no intersection: valid edge
                edges[src].add(dst)
                edges[dst].add(src)

    return edges

def closest_region_to_point(regions: list[Region], p: PointLike) -> int:
    p = Point(*p)

    min_index = -1
    min_dist = math.inf

    for i, region in enumerate(regions):
        centre = Point(*region.get_centre())
        dist = abs(p - centre)
        if dist < min_dist:
            min_index = i
            min_dist = dist
    
    return min_index
