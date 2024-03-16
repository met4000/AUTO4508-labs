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

def SplineDrive(*, x: int, y: int, alpha: int):
    lin_speed = 400
    lookahead = 0.06
    end_threshold = 0.06
    kp = 1.6

    ax = 0
    ay = 0
    bx = x
    by = y

    path_length = math.sqrt(x**2 + y**2)
    v_k = 2
    v_ax = path_length * v_k
    v_ay = 0
    v_bx = path_length * v_k * math.cos(alpha)
    v_by = path_length * v_k * math.sin(alpha)

    VWSetPosition(x=0, y=0, phi=0)

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

        dx = tracking_x - current_x
        dy = tracking_y - current_y
        if tracking_u > 1.0 and math.sqrt(dx**2 + dy**2) / path_length < end_threshold:
            break

        tracking_bearing = math.atan2(dy, dx) * 180 / math.pi
        err_angle = tracking_bearing - current_bearing

        ang_speed = kp * err_angle
        VWSetSpeed(lin_speed=lin_speed, ang_speed=round(ang_speed))

    VWStop()

from eye import SIMSetRobot
SIMSetRobot(0, 225, 210, 4, 0)

SplineDrive(x=1000, y=1500, alpha=0)

# SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
