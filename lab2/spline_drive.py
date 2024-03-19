from eyepy import *

def H1(u):
    return 2*(u**3) - 3*(u**2) + 1

def H2(u):
    return -2*(u**3) + 3*(u**2)

def H3(u):
   return u**3 - 2*(u**2) + u

def H4(u):
    return u**3 - u**2

def SplineDriveAbs(*, x: int, y: int, alpha: int, print_colour: Colour = RED):
    lin_speed = 300
    ang_speed = 60
    n_steps = 20

    start_pos, start_bearing_degs = VWGetPosition().as_float()
    target_pos, target_bearing_degs = Point(x, y), alpha

    start_bearing_rads = deg_to_rad(start_bearing_degs)
    target_bearing_rads = deg_to_rad(target_bearing_degs)

    v_len = 2 * abs(target_pos - start_pos)
    start_orientation = Vector.from_angle(start_bearing_rads) * v_len
    target_orientation = Vector.from_angle(target_bearing_rads) * v_len

    def spline(u: float) -> Point:
        return H1(u) * start_pos + H2(u) * target_pos + H3(u) * start_orientation + H4(u) * target_orientation
    
    points: list[Point] = [spline(u / n_steps) for u in range(n_steps + 1)]
    for point in points:
        LCDPixel((point / 10).round(), print_colour)
    for point in points[1:]:
        tracking_pos = point
        current_pos, current_bearing_degs = VWGetPosition().as_float()

        offset = tracking_pos - current_pos
        offset_distance = abs(offset)

        tracking_bearing_degs = rad_to_deg(offset.get_angle())
        err_angle_degs = tracking_bearing_degs - current_bearing_degs

        VWTurn(round(err_angle_degs), ang_speed=ang_speed)
        VWWait()
        VWStraight(round(offset_distance), lin_speed=lin_speed)
        VWWait()

    VWTurn(round(target_bearing_degs - VWGetPosition().phi), ang_speed=ang_speed)
    VWWait()

def SplineDrive(*, dx: int, dy: int, alpha: int, print_colour: Colour = RED):
    VWSetPosition((0, 0), 0)
    SplineDriveAbs(x=dx, y=dy, alpha=alpha, print_colour=print_colour)
