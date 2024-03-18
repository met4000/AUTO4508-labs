import math

from eyepy import *

def H1(u: float) -> float:
    return 2*(u**3) - 3*(u**2) + 1

def H2(u: float) -> float:
    return -2*(u**3) + 3*(u**2)

def H3(u: float) -> float:
   return u**3 - 2*(u**2) + u

def H4(u: float) -> float:
    return u**3 - u**2

def SplineDriveAbs(*, x: int, y: int, alpha: int):
    lin_speed = 400
    lookahead = 0.07
    end_threshold = 0.07 * 2
    kp = 1.6

    start_x, start_y, start_bearing_degs = VWGetPosition()
    target_x, target_y, target_bearing_degs = x, y, alpha

    dx = target_x - start_x
    dy = target_y - start_y

    start_bearing_rads = start_bearing_degs / 180 * math.pi
    target_bearing_rads = target_bearing_degs / 180 * math.pi

    v_len = 2 * math.sqrt(dx**2 + dy**2)
    start_v_x = v_len * math.cos(start_bearing_rads)
    start_v_y = v_len * math.sin(start_bearing_rads)
    target_v_x = v_len * math.cos(target_bearing_rads)
    target_v_y = v_len * math.sin(target_bearing_rads)

    def spline(u: float) -> tuple[float, float]:
        return (
            H1(u) * start_x + H2(u) * target_x + H3(u) * start_v_x + H4(u) * target_v_x,
            H1(u) * start_y + H2(u) * target_y + H3(u) * start_v_y + H4(u) * target_v_y
        )
    
    approx_path_length = v_len / 2 * 0.9
    path_duration_ms = approx_path_length / lin_speed * 1000
    initial_ms = OSGetCount()
    while True:
        current_ms = OSGetCount() - initial_ms
        current_u = current_ms / path_duration_ms
        tracking_u = current_u + lookahead

        tracking_x, tracking_y = spline(min(tracking_u, 1.0))
        current_x, current_y, current_bearing_degs = VWGetPosition()

        offset_x = tracking_x - current_x
        offset_y = tracking_y - current_y
        # print(tracking_u, math.sqrt(offset_x**2 + offset_y**2) / approx_path_length, end_threshold)
        if tracking_u > 1.0 and math.sqrt(offset_x**2 + offset_y**2) / approx_path_length < end_threshold:
            break

        tracking_bearing_degs = math.atan2(offset_y, offset_x) * 180 / math.pi
        err_angle_degs = tracking_bearing_degs - current_bearing_degs

        ang_speed = kp * err_angle_degs
        VWSetSpeed(lin_speed=lin_speed, ang_speed=round(ang_speed))

    VWStop()

    VWTurn(round(target_bearing_degs - VWGetPosition().phi), ang_speed=90)

from eye import SIMSetRobot
VWStop()
SIMSetRobot(0, 180, 180, 4, 0)

with open("way.txt") as file:
    points: list[Point] = [Point(*map(int, point.strip().split(" "))) for point in file]
    
points_and_bearings: list[tuple[Point, int]] = []
for i in range(len(points)):
    bearing_vector = points[(i + 1) % len(points)] - points[i - 1]
    bearing_rads = math.atan2(bearing_vector.y, bearing_vector.x)
    bearing_degs = bearing_rads / math.pi * 180
    points_and_bearings.append((points[i], bearing_degs))

# (x, y), alpha = points_and_bearings[0]
# SplineDriveAbs(x=x, y=y, alpha=alpha)
for (x, y), alpha in points_and_bearings:
    SplineDriveAbs(x=x, y=y, alpha=alpha)
