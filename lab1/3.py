from math import inf
from typing import Optional
from eyepy import *

def turn_to_closest(*, turn_speed = 100, encoder: EncoderPort = 1, left_motor: MotorPort = 1, right_motor: MotorPort = 3):
    ENCODERReset(encoder)
    MOTORDualDrive(left_motor, right_motor, offset=turn_speed)

    start_time = OSGetCount()
    min_dst = inf
    min_dst_time = start_time
    while OSGetCount() - start_time < 12000:
        front_distance = PSDGet(PSD_FRONT)
        if front_distance < min_dst:
            min_dst = front_distance
            min_dst_time = OSGetCount()
    final_time = OSGetCount()
    MOTORDualDrive(left_motor, right_motor, speed=0)

    MOTORDualDrive(left_motor, right_motor, offset=turn_speed)
    while OSGetCount() - final_time < final_time - min_dst_time: pass
    MOTORDualDrive(left_motor, right_motor, speed=0)

def follow_wall_until_distance(stop_distance_mm: int, *, wall_distance_mm: Optional[int] = None, target_speed = 80, psd: PSDPort = PSD_RIGHT, left_motor: MotorPort = 1, right_motor: MotorPort = 3):
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
        MOTORDualDrive(left_motor, right_motor, speed=target_speed, offset=offset)

        last_value = side_distance_mm

    MOTORDualDrive(left_motor, right_motor, speed=0)

def turn_left_90(left_motor: MotorPort = 1, right_motor: MotorPort = 3):
    MOTORDualDrive(left_motor, right_motor, offset=100)
    OSWait(3000)
    MOTORDualDrive(left_motor, right_motor, speed=0)


DISTANCE_FROM_WALL_MM = 1500
FRONT_TO_SIDE_PSD = 500

FORWARD_SPEED = 80
SCAN_TURN_SPEED = 100


# get PSD in range of a wall
MOTORDualDrive(1, 3, speed=100)
while all([dst > 7500 for dst in [PSDGet(PSD_FRONT), PSDGet(PSD_LEFT), PSDGet(PSD_RIGHT)]]): pass
MOTORDualDrive(1, 3, speed=0)

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

MOTORDualDrive(1, 3, speed=0)
