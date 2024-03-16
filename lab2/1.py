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
    n_steps = 10

    ax = 0
    ay = 0
    bx = x
    by = y

    path_length = 2 * math.sqrt(x**2 + y**2)
    v_ax = path_length
    v_ay = 0
    v_bx = path_length * math.cos(alpha)
    v_by = path_length * math.sin(alpha)

    VWSetPosition(x=0, y=0, phi=0)

    path: list[tuple[float, float]] = []
    for u in range(n_steps + 1):
        u /= n_steps
        path.append((
            H1(u) * ax + H2(u) * bx + H3(u) * v_ax + H4(u) * v_bx,
            H1(u) * ay + H2(u) * by + H3(u) * v_ay + H4(u) * v_by
        ))
    
    for i in range(len(path)):
        sp_x = path[i][0]
        sp_y = path[i][1]

        current_x, current_y, current_phi = VWGetPosition()

        err_x = sp_x - current_x
        err_y = sp_y - current_y
        target_phi = math.atan2(err_y, err_x) * 180 / math.pi
        rotation = target_phi - current_phi

        rotation_radians = rotation / 180 * math.pi
        ratio = math.sin(rotation_radians / 2)
        radius = math.sqrt(err_x**2 + err_y**2) / 2 / ratio if ratio != 0 else 0

        distance = rotation_radians * radius
        VWCurve(dist=round(distance), angle=round(rotation), lin_speed=500)
        VWWait()

# SplineDrive(x=1500, y=-1000, alpha=90)

SplineDrive(x=1000, y=0, alpha=0)
# SplineDrive(x=0, y=1000, alpha=90)
# SplineDrive(x=0, y=1000, alpha=0)
# SplineDrive(x=-500, y=-10, alpha=0)
# SplineDrive(x=-500, y=200, alpha=-90)
