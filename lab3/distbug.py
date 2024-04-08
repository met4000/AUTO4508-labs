from __future__ import annotations
from eyepy import *

display_max_x, display_max_y = LCDGetSize()
def transform_point(p: Point | tuple[float, float]) -> IntPoint:
    p = Point(*p)
    transformed_point = ((1, 0), (0, -1)) @ p.as_vector()
    screen_point = Point(0, display_max_y - 1) + transformed_point
    return screen_point.round()

_lidar_print_refresh_delay = 100
_last_print_lidar_time = OSGetCount()
def print_lidar(distances: list[int], *, colour_map: Optional[dict[int, Colour]] = None, y_offset: int = 10):
    global _last_print_lidar_time
    current_time = OSGetCount()
    if current_time - _last_print_lidar_time < _lidar_print_refresh_delay:
        return
    _last_print_lidar_time = current_time

    LCDClear()
    graph_root = Point(display_max_x / 2, y_offset)
    for i, distance in enumerate(distances):
        point_root = graph_root + (i - len(distances) / 2, 0)

        colour = colour_map[i] if colour_map is not None else WHITE

        LCDLine(transform_point(point_root), transform_point(point_root + (0, distance / 20)), colour)

def distbug_abs(target_pos: Point, *, hit_distance: int = 140, lin_speed: int = 300, ang_speed: int = 60, end_threshold: int = 70) -> bool:
    """
    :param:`x` mm
    :param:`y` mm
    """

    # amount required to be clear to leave
    step_size = hit_distance * 2

    while True:
        # turn to target
        current_pos, current_bearing_deg = VWGetPosition().as_float()
        pos_offset = target_pos - current_pos
        
        target_bearing_rad = pos_offset.get_angle()
        bearing_offset_deg = rad_to_deg(target_bearing_rad) - current_bearing_deg

        VWTurn(round(bearing_offset_deg), ang_speed=ang_speed)
        VWWait()


        # drive to target
        LIDARSet(range=135, n_points=135)

        straight_kp = 10.0

        while True:
            current_pos, current_bearing_deg = VWGetPosition().as_float()
            pos_offset = target_pos - current_pos

            if abs(pos_offset) < end_threshold:
                VWStop()
                return True

            lidar_distances = LIDARGet()
            print_lidar(lidar_distances)
            LCDLine(transform_point((0, 10 + hit_distance / 20)), transform_point((display_max_x, 10 + hit_distance / 20)), RED)
            if not all(distance > hit_distance for distance in lidar_distances):
                break
            
            target_bearing_rad = pos_offset.get_angle()
            bearing_offset_rad = target_bearing_rad - deg_to_rad(current_bearing_deg)

            error = bearing_offset_rad / math.pi
            ang_speed_mult = error * straight_kp

            VWSetSpeed(lin_speed=lin_speed, ang_speed=round(ang_speed * ang_speed_mult))
        VWStop()


        # drive around obstacle
        LIDARSet(range=360, n_points=360)
        obstacle_driving = True
        min_target_dist = math.inf
        hit_pos, _ = VWGetPosition().as_float()
        moved_from_hit = False
        while obstacle_driving:
            # turn perpendicular to the wall

            lidar_distances = LIDARGet(range=90, n_points=90)
            print_lidar_colour_map = {i: WHITE for i in range(len(lidar_distances))}

            window_points = 5
            lidar_window_avg_distances = [sum(lidar_distances[(i + i2 - window_points // 2) % len(lidar_distances)] for i2 in range(window_points)) / window_points for i in range(len(lidar_distances))]
            min_window_index = min(range(len(lidar_window_avg_distances)), key=lidar_window_avg_distances.__getitem__)

            window_indices = [(min_window_index + i - window_points // 2) % len(lidar_distances) for i in range(window_points)]
            for i in window_indices:
                print_lidar_colour_map[i] = YELLOW

            print_lidar(lidar_distances, colour_map=print_lidar_colour_map)
            LCDLine(transform_point((0, 10 + hit_distance / 20)), transform_point((display_max_x, 10 + hit_distance / 20)), RED)

            VWTurn(90 - (min_window_index - len(lidar_distances) // 2), ang_speed=ang_speed)

            while not VWDone():
                lidar_distances = LIDARGet()
                print_lidar_colour_map = {i: WHITE for i in range(len(lidar_distances))}

                window_points = 5
                lidar_window_avg_distances = [sum(lidar_distances[(i + i2 - window_points // 2) % len(lidar_distances)] for i2 in range(window_points)) / window_points for i in range(len(lidar_distances))]
                min_window_index = min(range(len(lidar_window_avg_distances)), key=lidar_window_avg_distances.__getitem__)

                window_indices = [(min_window_index + i - window_points // 2) % len(lidar_distances) for i in range(window_points)]
                for i in window_indices:
                    print_lidar_colour_map[i] = YELLOW

                print_lidar(lidar_distances, colour_map=print_lidar_colour_map)
                LCDLine(transform_point((0, 10 + hit_distance / 20)), transform_point((display_max_x, 10 + hit_distance / 20)), RED)

            # wall following

            obstacle_front_kp = 0.04
            obstacle_angle_kp = 0.075
            obstacle_distance_kp = 0.002
            obstacle_kd = 0
            last_error = 0.5

            while True:
                lidar_distances = LIDARGet()
                print_lidar_colour_map = {i: WHITE for i in range(len(lidar_distances))}
                
                # find distance to next object in front of target

                current_pos, current_bearing_deg = VWGetPosition().as_float()
                pos_offset = target_pos - current_pos

                target_dist = abs(pos_offset)
                min_target_dist = min(min_target_dist, target_dist)

                if target_dist < end_threshold:
                    VWStop()
                    return True
                
                hit_pos_dist = abs(hit_pos - current_pos)
                if hit_pos_dist < end_threshold:
                    if moved_from_hit:
                        VWStop()
                        return False
                else:
                    moved_from_hit = True
                
                target_bearing_rad = pos_offset.get_angle()
                bearing_offset_deg = rad_to_deg(target_bearing_rad) - current_bearing_deg

                n_target_lidar_points = 160
                target_lidar_index = 180 - round(bearing_offset_deg)
                target_lidar_indices = [(i + 360) % 360 for i in range(target_lidar_index - n_target_lidar_points // 2, target_lidar_index + n_target_lidar_points // 2)]
                target_lidar_distances = [lidar_distances[i] for i in target_lidar_indices]
                for i in target_lidar_indices:
                    print_lidar_colour_map[i] = RED
                
                target_lidar_distance = min(target_lidar_distances)
                if target_dist - target_lidar_distance <= min_target_dist - step_size:
                    obstacle_driving = False
                    VWStop()
                    break


                # wall follow

                n_front_points = 40
                front_distances = lidar_distances[180 - n_front_points // 2 : 180 + n_front_points // 2]
                for i in range(180 - n_front_points // 2, 180 + n_front_points // 2):
                    print_lidar_colour_map[i] = CYAN if print_lidar_colour_map[i] == WHITE else ORANGE

                n_side_points = 90
                side_distances = lidar_distances[270 - n_side_points // 2 : 270 + n_side_points // 2]
                for i in range(270 - n_side_points // 2, 270 + n_side_points // 2):
                    print_lidar_colour_map[i] = CYAN if print_lidar_colour_map[i] == WHITE else ORANGE

                print_lidar(lidar_distances, colour_map=print_lidar_colour_map)
                LCDLine(transform_point((0, 10 + hit_distance / 20)), transform_point((display_max_x, 10 + hit_distance / 20)), RED)
                LCDLine(transform_point((0, 10 + step_size / 20)), transform_point((display_max_x, 10 + step_size / 20)), GREEN)

                front_distance = min(front_distances)
                if front_distance < hit_distance:
                    VWStop()
                    break
                front_error = 1 / (front_distance - hit_distance)

                side_distance = sum(side_distances) // len(side_distances)
                min_side_index = min(range(len(side_distances)), key=side_distances.__getitem__)
                angle_error = len(side_distances) // 2 - min_side_index + 1

                wall_target_distance = hit_distance
                distance_error = wall_target_distance - side_distance

                error = front_error * obstacle_front_kp + angle_error * obstacle_angle_kp + distance_error * obstacle_distance_kp
                ang_speed_mult = error - (error - last_error) * obstacle_kd
                last_error = error

                VWSetSpeed(lin_speed=lin_speed, ang_speed=round(ang_speed * ang_speed_mult))

def distbug(dx: int, dy: int, *, hit_distance: int = 140, lin_speed: int = 300, ang_speed: int = 60, end_threshold: int = 70) -> bool:
    """
    :param:`dx` mm
    :param:`dy` mm
    """
    start_pos, start_bearing = VWGetPosition().as_float()
    v = Vector(dx, dy)
    start_target_vector = Vector.from_polar(magnitude=abs(v), angle=start_bearing + v.get_angle())

    target_pos = start_pos + start_target_vector
    return distbug_abs(target_pos, hit_distance=hit_distance, lin_speed=lin_speed, ang_speed=ang_speed, end_threshold=end_threshold)
