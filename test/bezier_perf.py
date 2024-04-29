from bezier import *

bezier = Bezier((100, 150), (250, 50), (400, 250))

for _ in range(1000):
    bezier_mask(bezier, radius=15)
