from eyepy import *

from bezier import *

bezier = quad_bezier((100, 150), (250, 50), (400, 250))

for _ in range(1000):
    bezier_mask(bezier, radius=15)
