from enum import auto
import random
from eyepy import *


# cheat and auto align the robot when starting
START_ALIGNED = True

ADD_FREE = False

# world units per block
X_RESOLUTION = 70
Y_RESOLUTION = 70

SEARCH_LIN_SPEED = 600 # mm/s
SEARCH_TURN_SPEED = 90 # deg/s
SEARCH_LIDAR_RANGE = 100 # deg
SEARCH_LIDAR_POINTS = 15

SEARCH_OBSTACLE_MIN_DIST = 200 # mm

SEARCH_SCAN_INTERVAL = 1800 # ms

SCAN_LIDAR_RANGE = 360 # deg
SCAN_LIDAR_POINTS = 360

SCAN_MAX_VALUE = 9000 # ignore values larger than this amount
SCAN_FREE_DIST = 1000 # world units


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

# TODO improve free area
def update_from_lidar():
    lidar_distances = LIDARGet(range=SCAN_LIDAR_RANGE, n_points=SCAN_LIDAR_POINTS)

    curr_pos, curr_bearing_deg = VWGetPosition().as_float()

    min_dist = SCAN_FREE_DIST

    for i, dist in enumerate(lidar_distances):
        if dist > SCAN_MAX_VALUE: continue

        point_rel_bearing_deg = lidar_rel_bearing(i, range=SCAN_LIDAR_RANGE, n_points=SCAN_LIDAR_POINTS)
        point_bearing_rad = deg_to_rad(curr_bearing_deg + point_rel_bearing_deg)

        point_vector = Vector.from_polar(magnitude=dist, angle=point_bearing_rad)
        point = curr_pos + point_vector

        dist = abs(curr_pos - point)
        if dist < min_dist: min_dist = dist

        block = IntPoint(math.floor(point.x / X_RESOLUTION), math.floor(point.y / Y_RESOLUTION))

        update_block_state(block, BlockState.OCCUPIED)
    
    if ADD_FREE:
        curr_block = IntPoint(math.floor(curr_pos.x / X_RESOLUTION), math.floor(curr_pos.y / Y_RESOLUTION))
        for dx, dy in itertools.product(
            range(math.floor(-min_dist / X_RESOLUTION), math.ceil(min_dist / X_RESOLUTION) + 1),
            range(math.floor(-min_dist / Y_RESOLUTION), math.ceil(min_dist / Y_RESOLUTION) + 1),
        ):
            block_offset = IntVector(dx, dy)
            offset = Vector(dx * X_RESOLUTION, dy * Y_RESOLUTION)

            if abs(offset) >= min_dist - abs(Vector(X_RESOLUTION, Y_RESOLUTION)): continue
            
            deg_diff = abs(rad_to_deg(offset.get_angle()) - curr_bearing_deg)
            if deg_diff > 180: deg_diff = 360 - deg_diff
            if deg_diff > SCAN_LIDAR_RANGE // 2 - 1: continue

            block = curr_block + block_offset
            update_block_state(block, BlockState.FREE)

VWStop()
if START_ALIGNED:
    from eye import SIMGetRobot, SIMSetRobot
    x, y, z, _ = SIMGetRobot(0)
    SIMSetRobot(0, x, y, z, 0)
VWSetPosition((0, 0), 0)

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

            turn_amount = random.randint(80, 150)
            VWTurn(round(turn_amount), ang_speed=SEARCH_TURN_SPEED)

            VWWait()
            break
    
    VWStop()
