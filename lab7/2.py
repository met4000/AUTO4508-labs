from enum import auto
import random
from eyepy import *


# cheat and auto align the robot when starting
START_ALIGNED = True

# world units per block
X_RESOLUTION = 10
Y_RESOLUTION = 10

SEARCH_LIN_SPEED = 400 # mm/s
SEARCH_TURN_SPEED = 90 # deg/s
SEARCH_LIDAR_RANGE = 140 # deg
SEARCH_LIDAR_POINTS = 15

SEARCH_OBSTACLE_MIN_DIST = 400 # mm
SEARCH_NEW_PATH_MIN_DIST = 800 # mm

SEARCH_SCAN_INTERVAL = 1500 # ms

SCAN_LIDAR_RANGE = 180 # deg
SCAN_LIDAR_POINTS = 180

SCAN_MAX_VALUE = 9000 # ignore values larger than this amount
SCAN_FREE_DIST = 300 # world units


class BlockState(Enum):
    UNKNOWN = auto()
    OCCUPIED = auto()
    FREE = auto()

state_cols = {
    # BlockState.UNKNOWN: BLACK,
    BlockState.OCCUPIED: RED,
    BlockState.FREE: WHITE,
}


# initialise with the spot occupied by the robot as free
block_states: dict[IntPoint, BlockState] = { IntPoint(0, 0): BlockState.FREE }
map_min_x = 0
map_max_x = 0
map_min_y = 0
map_max_y = 0

def update_block_state(block: IntPoint, state: BlockState):
    block_states[block] = state

    global map_min_x
    if block.x < map_min_x:
        map_min_x = block.x

    global map_max_x
    if block.x > map_max_x:
        map_max_x = block.x

    global map_min_y
    if block.y < map_min_y:
        map_min_y = block.y

    global map_max_y
    if block.y > map_max_y:
        map_max_y = block.y

def print_block_states():
    LCDClear()
    LCDSetPointMap(lcd_make_coord_map((map_max_x + 1, map_max_y + 1), (map_min_x - 1, map_min_y - 1)))

    for block, state in block_states.items():
        LCDPixelArea(block, state_cols[state])

def lidar_rel_bearing(i: int, *, range: int, n_points: int) -> float:
    """deg"""
    return ((n_points // 2) - i) / n_points * range

# TODO set some area as free
def update_from_lidar():
    lidar_distances = LIDARGet(range=SCAN_LIDAR_RANGE, n_points=SCAN_LIDAR_POINTS)

    curr_pos, curr_bearing_deg = VWGetPosition().as_float()

    min_x = curr_pos.x - SCAN_FREE_DIST
    max_x = curr_pos.x + SCAN_FREE_DIST
    min_y = curr_pos.y - SCAN_FREE_DIST
    max_y = curr_pos.y + SCAN_FREE_DIST

    for i, dist in enumerate(lidar_distances):
        if dist > SCAN_MAX_VALUE: continue

        point_rel_bearing_deg = lidar_rel_bearing(i, range=SCAN_LIDAR_RANGE, n_points=SCAN_LIDAR_POINTS)
        point_bearing_rad = deg_to_rad(curr_bearing_deg + point_rel_bearing_deg)

        point_vector = Vector.from_polar(magnitude=dist, angle=point_bearing_rad)
        point = curr_pos + point_vector

        if point.x > min_x: min_x = point.x + 1
        if point.x < max_x: max_x = point.x - 1
        if point.y > min_y: min_y = point.y + 1
        if point.y < max_y: max_y = point.y - 1

        block = IntPoint(math.floor(point.x / X_RESOLUTION), math.floor(point.y / Y_RESOLUTION))

        update_block_state(block, BlockState.OCCUPIED)
    
    for x in range(math.ceil(min_x), math.floor(max_x + 1)):
        for y in range(math.ceil(min_y), math.floor(max_y + 1)):
            block = IntPoint(math.floor(x / X_RESOLUTION), math.floor(y / Y_RESOLUTION))
            update_block_state(block, BlockState.FREE)

VWStop()
if START_ALIGNED:
    from eye import SIMGetRobot, SIMSetRobot
    x, y, z, _ = SIMGetRobot(0)
    SIMSetRobot(0, x, y, z, 0)
VWSetPosition((0, 0), 0)
left_turn = True

while True:
    update_from_lidar()
    print_block_states()

    forward_start_time = OSGetCount()
    VWSetSpeed(lin_speed=SEARCH_LIN_SPEED, ang_speed=0)
    while True:
        if OSGetCount() - forward_start_time > SEARCH_SCAN_INTERVAL:
            # time for a new scan
            break
        
        if min(LIDARGet(range=SEARCH_LIDAR_RANGE, n_points=SEARCH_LIDAR_POINTS)) <= SEARCH_OBSTACLE_MIN_DIST:
            # obstacle
            VWStop()

            turn_sign = 1 if left_turn else -1

            lidar_dists = LIDARGet(range=360, n_points=360)
            valid_indices = set(range(len(lidar_dists)))
            for i, dist in enumerate(lidar_dists):
                if (i - len(lidar_dists) // 2) * turn_sign > 0:
                    valid_indices.discard(i)

                if dist < SEARCH_NEW_PATH_MIN_DIST:
                    # ? eliminate adjacent by some amount of degrees
                    valid_indices.difference_update([i + delta for delta in range(-10, 10 + 1)])

                if dist <= SEARCH_LIDAR_RANGE:
                    # ? eliminate adjacent by some amount of degrees
                    valid_indices.difference_update([i + delta for delta in range(-SEARCH_LIDAR_RANGE//2, SEARCH_LIDAR_RANGE//2 + 1)])
            valid_turns: list[float] = [lidar_rel_bearing(i, range=360, n_points=360) for i in valid_indices]

            turn_amount = random.choice(valid_turns)
            VWTurn(round(turn_amount), ang_speed=SEARCH_TURN_SPEED)

            left_turn = not left_turn
            VWWait()
            break
    
    VWStop()
