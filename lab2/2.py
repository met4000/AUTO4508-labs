from __future__ import annotations
import random

from eyepy import *

from spline_drive import SplineDriveAbs


from eye import SIMSetRobot
VWStop()
SIMSetRobot(0, 180, 180, 4, 0)

with open("./way.txt") as file:
    points: list[Point] = [Point(*map(int, point.strip().split(" "))) for point in file]
    
points_and_bearings: list[tuple[tuple[int, int], int]] = []
for i in range(len(points)):
    bearing_vector = points[(i + 1) % len(points)] - points[i - 1]
    bearing_degs = rad_to_deg(bearing_vector.get_angle())
    points_and_bearings.append((points[i].round(), round(bearing_degs)))

# for printing splines to LCD
last_colour: Colour = WHITE
colours: set[Colour] = {RED, GREEN, CYAN, YELLOW, PURPLE}

i = 0
while True:
    colour: Colour = random.choice(tuple(colours.difference((last_colour,))))
    last_colour = colour
    
    (x, y), alpha = points_and_bearings[i % len(points_and_bearings)]
    SplineDriveAbs(x=x, y=y, alpha=alpha, print_colour=colour)
    i += 1
