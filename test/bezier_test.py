from eyepy import *

from bezier import *


BEZIER_COL = RED
MASK_COL = GREEN


bezier = quad_bezier((100, 150), (250, 50), (400, 150))
curve_pixels = set(bezier_pixels(bezier))
for pixel in curve_pixels:
    LCDPixel(pixel, BEZIER_COL)

mask = bezier_mask(bezier, radius=10)
for pixel in mask:
    if pixel in curve_pixels: continue
    
    LCDPixel(pixel, MASK_COL)

print()
