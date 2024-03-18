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

    ax = 0
    ay = 0
    bx = dx
    by = dy

    final_angle = alpha

    v_len = 2 * math.sqrt(dx**2 + dy**2)
    v_ax = v_len
    v_ay = 0
    v_bx = v_len * math.cos(alpha)
    v_by = v_len * math.sin(alpha)

    VWSetPosition(x=0, y=0, phi=0)

    def spline(u: float) -> tuple[float, float]:
        return (
            H1(u) * ax + H2(u) * bx + H3(u) * v_ax + H4(u) * v_bx,
            H1(u) * ay + H2(u) * by + H3(u) * v_ay + H4(u) * v_by
        )
    
    approx_path_length = v_len / 2
    path_duration_ms = approx_path_length / lin_speed * 1000
    initial_ms = OSGetCount()
    while True:
        current_ms = OSGetCount() - initial_ms
        current_u = current_ms / path_duration_ms
        tracking_u = current_u + lookahead

        tracking_x, tracking_y = spline(min(tracking_u, 1.0))
        current_x, current_y, current_bearing = VWGetPosition()

        offset_x = tracking_x - current_x
        offset_y = tracking_y - current_y
        if tracking_u > 1.0 and math.sqrt(offset_x**2 + offset_y**2) / approx_path_length < end_threshold:
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
SplineDrive(dx=500, dy=1500, alpha=-145)

# SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
