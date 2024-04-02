from eyepy import *

from distbug import distbug


VWStop()
from eye import SIMSetRobot
SIMSetRobot(0, 500, 500, 4, 0)

VWSetPosition((0, 0), 0)
distbug(3000, 3000)
