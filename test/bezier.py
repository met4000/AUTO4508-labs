from eyepy import *

Bezier = Callable[[float], Point]

def quad_bezier(p0: PointLike, p1: PointLike, p2: PointLike) -> Bezier:
    p0 = Point(*p0)
    p1 = Point(*p1)
    p2 = Point(*p2)

    def f(t: float) -> Point:
        t_prime = 1 - t
        return t_prime ** 2 * p0 + 2 * t_prime * t * p1 + t ** 2 * p2
    
    return f

def approx_bezier_length(bezier: Bezier):
    p_start = bezier(0)
    p_mid = bezier(0.5)
    p_end = bezier(1)

    return abs(p_end - p_mid) + abs(p_mid - p_start)

def bezier_points(bezier: Bezier, n_points: int) -> list[Point]:
    t_values = [i / n_points for i in range(n_points + 1)]
    return [bezier(t) for t in t_values]

def nearest_pixel(p: PointLike) -> IntPoint:
    return Point(*p).round()

def bezier_pixels(bezier: Bezier) -> list[IntPoint]:
    approx_length = approx_bezier_length(bezier)
    points: list[Point] = bezier_points(bezier, math.ceil(approx_length * 3))

    pixels: list[IntPoint] = []
    pixels.append(nearest_pixel(points[0]))
    for point in points:
        new_pixel = nearest_pixel(point)
        if new_pixel == pixels[-1]: continue

        pixels.append(new_pixel)
    
    return pixels

def bezier_mask(bezier: Bezier, *, radius: float, epsilon: float = 0.001) -> set[IntPoint]:
    centre_points = [Point(*p) for p in bezier_pixels(bezier)]

    pixels: set[IntPoint] = set()

    offsets = [IntVector(dx, dy) for dx, dy in itertools.product(range(-1, 1 + 1), range(-1, 1 + 1)) if not (dx == 0 and dy == 0)]
    offset_pixels: dict[IntVector, set[IntVector]] = { offset: set() for offset in offsets }
    
    start_centre = centre_points[0]

    # compute entire initial circle, and also relative circles
    for dx, dy in itertools.product(
        range(math.floor(-radius) - 1, math.ceil(radius) + 1 + 1),
        range(math.floor(-radius) - 1, math.ceil(radius) + 1 + 1)
    ):
        test_offset = IntVector(dx, dy)
        test_p = start_centre.round() + test_offset

        if abs(start_centre - test_p) < radius + epsilon:
            pixels.add(test_p)
            continue

        for offset in offsets:
            offset_centre = start_centre + offset
            if abs(offset_centre - test_p) < radius + epsilon:
                offset_pixels[offset].add(test_offset - offset)
    
    for prev_i, centre in enumerate(centre_points[1:]):
        prev_centre = centre_points[prev_i]
        centre_diff = (centre - prev_centre).round()

        new_pixels = [(centre + offset).round() for offset in offset_pixels[centre_diff]]

        pixels.update(new_pixels)

    return pixels
