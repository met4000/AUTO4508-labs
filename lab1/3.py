from math import inf
import time
from typing import Optional
from eye import * # type: ignore

def VWStop(*, left_motor, right_motor):
    MOTORDrive(left_motor, 0)
    MOTORDrive(right_motor, 0)

def turn_to_closest(*, turn_speed = 100, encoder = 1, left_motor = 1, right_motor = 3):
    ENCODERReset(encoder)
    MOTORDrive(left_motor, -turn_speed)
    MOTORDrive(right_motor, turn_speed)

    start_time = round(time.time()*1000)
    min_dst = inf
    min_dst_time = start_time
    while round(time.time()*1000) - start_time < 12000:
        front_distance = PSDGet(PSD_FRONT)
        if front_distance < min_dst:
            min_dst = front_distance
            min_dst_time = round(time.time()*1000)
    final_time = round(time.time()*1000)
    VWStop(left_motor=left_motor, right_motor=right_motor)

    MOTORDrive(left_motor, turn_speed)
    MOTORDrive(right_motor, -turn_speed)
    while round(time.time()*1000) - final_time < final_time - min_dst_time: pass
    VWStop(left_motor=left_motor, right_motor=right_motor)

def clamp(n, min_n = -100, max_n = 100):
    return max(min(n, max_n), min_n)

def follow_wall_until_distance(stop_distance_mm: int, *, wall_distance_mm: Optional[int] = None, target_speed = 80, psd = PSD_RIGHT, left_motor = 1, right_motor = 3):
    target_distance_mm = wall_distance_mm if wall_distance_mm else PSDGet(psd)

    kp = 0.04
    ki = 0.00035
    kd = 4

    err_sum = 0
    last_value = PSDGet(psd)

    while PSDGet(PSD_FRONT) > stop_distance_mm:
        side_distance_mm = PSDGet(psd)

        error = target_distance_mm - side_distance_mm
        err_sum += error
        delta = last_value - side_distance_mm

        offset = error * kp + err_sum * ki + delta * kd
        MOTORDrive(left_motor, clamp(round(target_speed - offset)))
        MOTORDrive(right_motor, clamp(round(target_speed + offset)))

        last_value = side_distance_mm

    VWStop(left_motor=left_motor, right_motor=right_motor)

def turn_left_90(left_motor = 1, right_motor = 3):
    MOTORDrive(left_motor, -100)
    MOTORDrive(right_motor, 100)
    OSWait(3000)
    VWStop(left_motor=left_motor, right_motor=right_motor)


DISTANCE_FROM_WALL_MM = 1500
FRONT_TO_SIDE_PSD = 500

FORWARD_SPEED = 80
SCAN_TURN_SPEED = 100


# get PSD in range of a wall
MOTORDrive(1, 100)
MOTORDrive(3, 100)
while all([dst > 7500 for dst in [PSDGet(PSD_FRONT), PSDGet(PSD_LEFT), PSDGet(PSD_RIGHT)]]): pass
VWStop(left_motor=1, right_motor=3)

# go to nearest wall
turn_to_closest(turn_speed=SCAN_TURN_SPEED)
follow_wall_until_distance(DISTANCE_FROM_WALL_MM - FRONT_TO_SIDE_PSD, target_speed=FORWARD_SPEED)

# go to a corner
turn_left_90()
follow_wall_until_distance(DISTANCE_FROM_WALL_MM - FRONT_TO_SIDE_PSD, target_speed=FORWARD_SPEED, wall_distance_mm=DISTANCE_FROM_WALL_MM)
turn_left_90()

# loop
for _ in range(4):
    follow_wall_until_distance(DISTANCE_FROM_WALL_MM - FRONT_TO_SIDE_PSD, target_speed=FORWARD_SPEED, wall_distance_mm=DISTANCE_FROM_WALL_MM)
    turn_left_90()

VWStop(left_motor=1, right_motor=3)
