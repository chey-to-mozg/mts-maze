from enum import IntEnum


class Directions(IntEnum):
    up = 0
    right = 1
    down = 2
    left = 3


MOUSE_ID = ""
DEBUG_LOGGING = False

# Mouse related
SENSING_OFFSET = 40
ROTATION_SPEED = 220
FORWARD_SPEED = 70
FRONT_REFERENCE = 70
ANGLE_OFFSET = 10
KP_ROT = 4.0
KP_STEERING = 0.4
MAX_SPEED = 200
PRE_TURN_SPEED = 100

LEFT_REFERENCE = 59
RIGHT_REFERENCE = 49

WALL_THRESHOLD = 130  # wall sensor data in center {'1': 239, '2': 59, '3': 82, '4': 225, '5': 70, '6': 93},
FRONT_WALL_THRESHOLD = 180
PRE_TURN_THRESHOLD = 110
CENTER_REFERENCE = 80

# Maze related
CELL = 160
HALF_CELL = CELL / 2

# all wall related information is stored in one uint8_t array, first 4 bits to mark walls visited (1 bit for 1 wall)
# last 4 bits for actual wall data (1 bit for 1 wall)

VISITED = 0xF0

UP_WALL_VISITED = 0b10000000
RIGHT_WALL_VISITED = 0b01000000
DOWN_WALL_VISITED = 0b00100000
LEFT_WALL_VISITED = 0b00010000

UP_WALL = 0b00001000
RIGHT_WALL = 0b00000100
DOWN_WALL = 0b00000010
LEFT_WALL = 0b00000001

WALLS_VISITED = (
    UP_WALL_VISITED,
    RIGHT_WALL_VISITED,
    DOWN_WALL_VISITED,
    LEFT_WALL_VISITED,
)
WALLS = (UP_WALL, RIGHT_WALL, DOWN_WALL, LEFT_WALL)
DIRECTION_TO_CHAR = ("^", ">", "v", "<")
