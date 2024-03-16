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

def SplineDrive(*, dx: int, dy: int, alpha: int):
    lin_speed = 400
    lookahead = 0.06
    end_threshold = 0.06
    kp = 1.6

    ax, ay, start_angle = VWGetPosition()
    start_angle_rads = start_angle * math.pi / 180
    bx = ax + dx * math.cos(start_angle_rads) + dy * math.sin(start_angle_rads)
    by = ay + dx * math.sin(start_angle_rads) + dy * math.cos(start_angle_rads)

    path_length = math.sqrt(dx**2 + dy**2)
    final_angle = start_angle + alpha
    final_angle_rads = final_angle * math.pi / 180

    v_k = 2
    v_ax = path_length * v_k * math.cos(start_angle_rads)
    v_ay = path_length * v_k * math.sin(start_angle_rads)
    v_bx = path_length * v_k * math.cos(final_angle_rads)
    v_by = path_length * v_k * math.sin(final_angle_rads)

    def spline(u: float) -> tuple[float, float]:
        return (
            H1(u) * ax + H2(u) * bx + H3(u) * v_ax + H4(u) * v_bx,
            H1(u) * ay + H2(u) * by + H3(u) * v_ay + H4(u) * v_by
        )
    
    path_duration_ms = path_length / lin_speed * 1000
    initial_ms = OSGetCount()
    while True:
        current_ms = OSGetCount() - initial_ms
        current_u = current_ms / path_duration_ms
        tracking_u = current_u + lookahead

        tracking_x, tracking_y = spline(min(tracking_u, 1.0))
        current_x, current_y, current_bearing = VWGetPosition()

        offset_x = tracking_x - current_x
        offset_y = tracking_y - current_y
        if tracking_u > 1.0 and math.sqrt(offset_x**2 + offset_y**2) / path_length < end_threshold:
            break

        tracking_bearing = math.atan2(offset_y, offset_x) * 180 / math.pi
        err_angle = tracking_bearing - current_bearing

        ang_speed = kp * err_angle
        VWSetSpeed(lin_speed=lin_speed, ang_speed=round(ang_speed))

    VWStop()

    VWTurn(final_angle - VWGetPosition().phi, ang_speed=90)

from eye import SIMSetRobot
SIMSetRobot(0, 225, 210, 4, 0)

# SplineDrive(x=1000, y=1000, alpha=0)
SplineDrive(dx=500, dy=1500, alpha=90)
OSWait(5000)
SplineDrive(dx=500, dy=1500, alpha=90)

# SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
