from math import inf
from typing import Optional
from eyepy import *


def turn_to_closest(*, angular_speed = 60, encoder: EncoderPort = 1):
    ENCODERReset(encoder)
    VWTurn(360, ang_speed=angular_speed)

    min_dst = inf
    min_dst_encoder = 0
    while not VWDone():
        front_distance = PSDGet(PSD_FRONT)
        if front_distance < min_dst:
            min_dst = front_distance
            min_dst_encoder = ENCODERRead(encoder)
    VWStop()
    final_encoder = ENCODERRead(encoder)

    target_angle: int = round(360 * (final_encoder - min_dst_encoder) / final_encoder % 360)
    if target_angle > 180: target_angle = 360 - target_angle

    VWTurn(target_angle, ang_speed=angular_speed); VWWait()

def follow_wall_until_distance(stop_distance_mm: int, *, wall_distance_mm: Optional[int] = None, speed = 200, psd: PSDPort = PSD_RIGHT):
    target_distance_mm = wall_distance_mm if wall_distance_mm else PSDGet(psd)

    kp = 10
    while PSDGet(PSD_FRONT) > stop_distance_mm:
        side_distance_mm = PSDGet(psd)
        error = target_distance_mm - side_distance_mm
        VWSetSpeed(lin_speed=speed, ang_speed=error * kp)
    VWStop()


DISTANCE_FROM_WALL_MM = 150
DISTANCE_BETWEEN_PASSES_MM = DISTANCE_FROM_WALL_MM * 2

FORWARD_SPEED = 200
FAST_TURN_SPEED = 90
SCAN_TURN_SPEED = 60


# go to nearest wall
turn_to_closest(angular_speed=SCAN_TURN_SPEED)
follow_wall_until_distance(DISTANCE_FROM_WALL_MM, speed=FORWARD_SPEED)

# go to a corner
VWTurn(-90, ang_speed=FAST_TURN_SPEED); VWWait()
follow_wall_until_distance(DISTANCE_FROM_WALL_MM, speed=FORWARD_SPEED)
VWTurn(-90, ang_speed=FAST_TURN_SPEED); VWWait()

# 'lawnmower' pattern
final_row = False
flip_turn = False
while not final_row:
    follow_wall_until_distance(DISTANCE_FROM_WALL_MM, speed=FORWARD_SPEED)
    VWTurn(-90 * (-1 if flip_turn else 1), ang_speed=FAST_TURN_SPEED); VWWait()

    VWStraight(DISTANCE_BETWEEN_PASSES_MM, lin_speed=FORWARD_SPEED)
    while not VWDone():
        if PSDGet(PSD_FRONT) < DISTANCE_FROM_WALL_MM:
            final_row = True
            break

    VWTurn(-90 * (-1 if flip_turn else 1), ang_speed=FAST_TURN_SPEED); VWWait()
    flip_turn = not flip_turn

# final row
follow_wall_until_distance(DISTANCE_FROM_WALL_MM, speed=FORWARD_SPEED)
