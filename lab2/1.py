import math

from eyepy import *

def H1(u):
    return 2*(u**3) - 3*(u**2) + 1

def H2(u):
    return -2*(u**3) + 3*(u**2)

def H3(u):
   return u**3 - 2*(u**2) + u

def H4(u):
    return u**3 - u**2

def SplineDriveAbs(*, x: int, y: int, alpha: int):
    lin_speed = 300
    ang_speed = 60
    n_steps = 20

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
    
    points: list[Point] = [Point(*spline(u / n_steps)) for u in range(n_steps + 1)]
    for point in points[1:]:
        tracking_x, tracking_y = point
        current_x, current_y, current_bearing_degs = VWGetPosition()

        offset_x = tracking_x - current_x
        offset_y = tracking_y - current_y
        offset_distance = math.sqrt(offset_x**2 + offset_y**2)

        tracking_bearing_degs = math.atan2(offset_y, offset_x) * 180 / math.pi
        err_angle_degs = tracking_bearing_degs - current_bearing_degs

        err_angle_degs = round(err_angle_degs) % 360
        VWTurn(360 - err_angle_degs if err_angle_degs > 180 else err_angle_degs, ang_speed=ang_speed)
        VWWait()
        VWStraight(round(offset_distance), lin_speed=lin_speed)
        VWWait()

    err_angle_degs = target_bearing_degs - VWGetPosition().phi
        
    err_angle_degs = round(err_angle_degs) % 360
    VWTurn(360 - err_angle_degs if err_angle_degs > 180 else err_angle_degs, ang_speed=ang_speed)
    VWWait()

def SplineDrive(*, dx: int, dy: int, alpha: int):
    VWSetPosition(x=0, y=0, phi=0)
    SplineDriveAbs(x=dx, y=dy, alpha=alpha)

from eye import SIMSetRobot
SIMSetRobot(0, 225, 210, 4, 0)

# SplineDrive(x=1000, y=1000, alpha=0)
SplineDrive(dx=500, dy=1500, alpha=-145)

# SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
