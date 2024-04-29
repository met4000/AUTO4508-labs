import itertools
import math
from typing import Callable


Bezier = Callable[[float], tuple[float, float]]

def quad_bezier(p0: tuple[float, float], p1: tuple[float, float], p2: tuple[float, float]) -> Bezier:
    def f(t: float) -> tuple[float, float]:
        t_prime = 1 - t
        x = t_prime**2 * p0[0] + 2 * t_prime * t * p1[0] + t**2 * p2[0]
        y = t_prime**2 * p0[1] + 2 * t_prime * t * p1[1] + t**2 * p2[1]
        return (x, y)
    
    return f

def approx_bezier_length(bezier: Bezier) -> float:
    p_start_x, p_start_y = bezier(0)
    p_mid_x, p_mid_y = bezier(0.5)
    p_end_x, p_end_y = bezier(1)

    section_1 = math.sqrt((p_mid_x - p_start_x)**2 + (p_mid_y - p_start_y)**2)
    section_2 = math.sqrt((p_end_x - p_mid_x)**2 + (p_end_y - p_mid_y)**2)

    return section_1 + section_2

def bezier_points(bezier: Bezier, n_points: int) -> list[tuple[float, float]]:
    t_values = [i / n_points for i in range(n_points + 1)]
    return [bezier(t) for t in t_values]

def nearest_pixel(p: tuple[float, float]) -> tuple[int, int]:
    return (round(p[0]), round(p[1]))

def bezier_pixels(bezier: Bezier) -> list[tuple[int, int]]:
    approx_length = approx_bezier_length(bezier)
    points: list[tuple[float, float]] = bezier_points(bezier, math.ceil(approx_length * 3))

    pixels: list[tuple[int, int]] = []
    pixels.append(nearest_pixel(points[0]))
    for point in points:
        new_pixel = nearest_pixel(point)
        if new_pixel == pixels[-1]: continue

        pixels.append(new_pixel)
    
    return pixels

def _compute_initial(start: tuple[int, int], *, radius: float, epsilon: float) -> tuple[set[tuple[int, int]], dict[tuple[int, int], set[tuple[int, int]]]]:
    radius_prime = radius + epsilon

    pixels: set[tuple[int, int]] = set()

    offsets = [offset for offset in itertools.product(range(-1, 1 + 1), range(-1, 1 + 1)) if offset != (0, 0)]
    offset_pixels: dict[tuple[int, int], set[tuple[int, int]]] = { offset: set() for offset in offsets }

    # compute entire initial circle, and also relative circles
    for dx, dy in itertools.product(
        range(math.floor(-radius) - 1, math.ceil(radius) + 1 + 1),
        range(math.floor(-radius) - 1, math.ceil(radius) + 1 + 1)
    ):
        test_p_x = start[0] + dx
        test_p_y = start[1] + dy

        if math.sqrt((start[0] - test_p_x)**2 + (start[1] - test_p_y)**2) < radius_prime:
            pixels.add((test_p_x, test_p_y))
            continue

        for offset in offsets:
            offset_centre_x = start[0] + offset[0]
            offset_centre_y = start[1] + offset[1]
            if math.sqrt((offset_centre_x - test_p_x)**2 + (offset_centre_y - test_p_y)**2) < radius_prime:
                offset_pixels[offset].add((dx - offset[0], dy - offset[1]))
    
    return pixels, offset_pixels

def _compute_mask(centres: list[tuple[int, int]], pixels: set[tuple[int, int]], offsets: dict[tuple[int, int], set[tuple[int, int]]]) -> set[tuple[int, int]]:
    for prev_i, (centre_x, centre_y) in enumerate(centres[1:]):
        centre_diff = (centre_x - centres[prev_i][0], centre_y - centres[prev_i][1])
        new_pixels = [(centre_x + offset[0], centre_y + offset[1]) for offset in offsets[centre_diff]]
        pixels.update(new_pixels)
    
    return pixels

def bezier_mask(bezier: Bezier, *, radius: float, epsilon: float = 0.001) -> set[tuple[int, int]]:
    centre_points = bezier_pixels(bezier)
    pixels, offsets = _compute_initial(centre_points[0], radius=radius, epsilon=epsilon)
    return _compute_mask(centre_points, pixels, offsets)
