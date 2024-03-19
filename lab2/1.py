from eyepy import *

from spline_drive import SplineDrive


from eye import SIMSetRobot
SIMSetRobot(0, 225, 210, 4, 0)

# SplineDrive(x=1000, y=1000, alpha=0)
SplineDrive(dx=600, dy=1200, alpha=-130)

# SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
