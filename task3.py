import requests
import queue
import time
import math
from enum import IntEnum
from typing import Optional, Union

my_token = '6801ebee-4378-4cea-a1b2-bc84e115bbbbc8a2aed2-c20d-442f-a162-2ed391facfca'
DEBUG_LOGGING = False

CELL = 166
HALF_CELL = CELL / 2
SENSING_OFFSET = 30
ROTATION_SPEED = 250
FORWARD_SPEED = 80
FRONT_REFERENCE = 70
ANGLE_OFFSET = 10
KP_ROT = 4.0
KP_STEERING = 0.4
MAX_SPEED = 150
PRE_TURN_SPEED = 100

LEFT_REFERENCE = 59
RIGHT_REFERENCE = 49

WALL_THRESHOLD = 130
FRONT_WALL_THRESHOLD = 210
PRE_TURN_THRESHOLD = 100
PRE_TURN_OFFSET = 30
CENTER_REFERENCE = 80


class Mouse:

    def __init__(self, token: str):
        super().__init__()
        self._api_route = 'http://127.0.0.1:8801/api/v1'
        self._token = token
        self.left_wall_distance = 0
        self.front_wall_distance = 0
        self.right_wall_distance = 0
        self.left_wall = False
        self.front_wall = False
        self.right_wall = False

        self.angle = 0
        self.prev_angle = 0
        self.reference_angle = 0
        self.prev_x = 0
        self.prev_y = 0
        self.offset_x = 0
        self.offset_y = 0
        self.pos = 0
        self.speed = 0
        self.max_speed = MAX_SPEED

        self.rot_error = 0
        self._steering_enabled = True

        self.update_sensor_data(init_update=True)
        self.reference_angle = self.angle
        self.prev_angle = self.angle

    def _move_motors(self, pwm_left: Union[int, float], pwm_right: Union[int, float], m_time: float = 0.1):
        pwm_left = int(pwm_left)
        pwm_right = int(pwm_right)
        requests.post(
            f'{self._api_route}/robot-motors/move?token={self._token}&l={pwm_left}&l_time={m_time}&r={pwm_right}&r_time={m_time}'
        )
        time.sleep(m_time)

    def _move(self, dist: float, stop_threshold: float):
        self.update_sensor_data()
        while self.pos < dist:
            if self.speed < self.max_speed:
                left_pwm = FORWARD_SPEED + self.rot_error
                right_pwm = FORWARD_SPEED - self.rot_error
                self._move_motors(left_pwm, right_pwm)
            else:
                time.sleep(0.1)
            self.update_sensor_data()
            if self.front_wall_distance < stop_threshold:
                return

    def move(self, dist: float, stop_threshold: float = PRE_TURN_THRESHOLD):
        self.pos = 0
        self._move(dist, stop_threshold)

    def wait_until_position(self, dist: float, stop_threshold: float = PRE_TURN_THRESHOLD):
        self._move(dist, stop_threshold)

    def stop(self):
        self.update_sensor_data()
        while self.speed > 5:
            self._move_motors(-FORWARD_SPEED, -FORWARD_SPEED)
            self.update_sensor_data()

    def left(self):
        self._steering_enabled = False
        self.max_speed = PRE_TURN_SPEED
        self.wait_until_position(CELL + PRE_TURN_OFFSET)
        self.reference_angle -= 90
        self.update_sensor_data()
        while self.angle > self.reference_angle + ANGLE_OFFSET:
            self._move_motors(-ROTATION_SPEED, ROTATION_SPEED)
            self.update_sensor_data()

        self.max_speed = MAX_SPEED
        self.move(30)
        self.pos = CELL
        self._steering_enabled = True

    def right(self):
        self._steering_enabled = False
        self.max_speed = PRE_TURN_SPEED
        self.wait_until_position(CELL + PRE_TURN_OFFSET)
        self.reference_angle += 90
        self.update_sensor_data()
        while self.angle < self.reference_angle - ANGLE_OFFSET:
            self._move_motors(ROTATION_SPEED, -ROTATION_SPEED)
            self.update_sensor_data()

        self.max_speed = MAX_SPEED
        self.move(30)
        self.pos = CELL
        self._steering_enabled = True

    def right_in_place(self):
        self._steering_enabled = False
        self.reference_angle += 90
        self.update_sensor_data()
        while self.angle < self.reference_angle - ANGLE_OFFSET:
            self._move_motors(ROTATION_SPEED, -ROTATION_SPEED)
            self.update_sensor_data()

        self._steering_enabled = True

    def left_in_place(self):
        self._steering_enabled = False
        self.reference_angle -= 90
        self.update_sensor_data()
        while self.angle > self.reference_angle + ANGLE_OFFSET:
            self._move_motors(-ROTATION_SPEED, ROTATION_SPEED)
            self.update_sensor_data()

        self._steering_enabled = True

    def around(self):
        self.left_in_place()
        self.left_in_place()

    def get_sensors(self):
        return requests.get(f'{self._api_route}/robot-cells/sensor-data?token={self._token}').json()

    def _calc_errors(self):
        ang_error = (self.reference_angle - self.angle) * KP_ROT

        pos_error = 0
        left_err = (self.left_wall_distance - LEFT_REFERENCE)
        right_err = (self.right_wall_distance - RIGHT_REFERENCE)
        if self.left_wall and self.right_wall:
            pos_error = left_err - right_err
        elif self.left_wall:
            pos_error = 2 * left_err
        elif self.right_wall:
            pos_error = 2 * right_err
        pos_error *= KP_STEERING

        if not self._steering_enabled:
            pos_error = 0

        self.rot_error = ang_error - pos_error

    def _calc_dist(self):
        return math.sqrt((self.prev_y - self.offset_y) ** 2 + (self.prev_x - self.offset_x) ** 2)

    def _update_movement(self):
        diff = self._calc_dist()
        self.speed = diff / 0.1
        self.pos += diff
        self.prev_y = self.offset_y
        self.prev_x = self.offset_x

    def update_sensor_data(self, init_update: bool = False):
        sensor_data = self.get_sensors()
        self.front_wall_distance = sensor_data['front_distance']
        if not init_update:
            self.left_wall_distance = sensor_data['left_45_distance']
            self.right_wall_distance = sensor_data['right_45_distance']
        else:
            self.left_wall_distance = sensor_data['left_side_distance']
            self.right_wall_distance = sensor_data['right_side_distance']
        self.left_wall = self.left_wall_distance < WALL_THRESHOLD
        self.front_wall = self.front_wall_distance < FRONT_WALL_THRESHOLD
        self.right_wall = self.right_wall_distance < WALL_THRESHOLD

        angle = sensor_data['rotation_yaw']
        if init_update:
            self.reference_angle = angle
            self.prev_angle = angle
            self.angle = angle
        else:
            if angle < -90:
                if self.prev_angle > 90:
                    self.prev_angle -= 360
            if angle > 90:
                if self.prev_angle < -90:
                    self.prev_angle += 360
            diff = angle - self.prev_angle
            self.angle += diff
            self.prev_angle = angle

        self.offset_x = sensor_data['down_x_offset']
        self.offset_y = sensor_data['down_y_offset']

        self._update_movement()

        self._calc_errors()

    def __str__(self):
        return (
            f'< {self.left_wall_distance} {self.left_wall} \n'
            f'^ {self.front_wall_distance} {self.front_wall} \n'
            f'> {self.right_wall_distance} {self.right_wall} \n'
            f'angle: {self.angle} ({self.reference_angle})\n'
            f'x: {self.offset_x} y: {self.offset_y}\n'
            f'error: {self.rot_error}'
        )


class Directions(IntEnum):
    up = 0
    right = 1
    down = 2
    left = 3


VISITED = 0xF0
UP_WALL_VISITED = 0b10000000
RIGHT_WALL_VISITED = 0b01000000
DOWN_WALL_VISITED = 0b00100000
LEFT_WALL_VISITED = 0b00010000
UP_WALL = 0b00001000
RIGHT_WALL = 0b00000100
DOWN_WALL = 0b00000010
LEFT_WALL = 0b00000001

WALLS_VISITED = (UP_WALL_VISITED, RIGHT_WALL_VISITED, DOWN_WALL_VISITED, LEFT_WALL_VISITED)
WALLS = (UP_WALL, RIGHT_WALL, DOWN_WALL, LEFT_WALL)
DIRECTION_TO_CHAR = ('^', '>', 'v', '<')


class Maze:
    shape = 16
    start = (15, 0)
    start_direction = Directions.up
    finish = [(8, 8), (7, 7), (7, 8), (8, 7)]

    NEIGHBOURS = (
        (shape - 1, 0),
        (0, 1),
        (1, 0),
        (0, shape - 1),
    )

    def __init__(self, token: str):
        self._api_route = 'http://127.0.0.1:8801/api/v1'
        self._token = token
        self.mouse_position = self.start
        self.mouse_direction = self.start_direction
        self.maze = []
        self.walls = []
        self.path = []
        self.reset_maze()

    @property
    def cur_x(self):
        return self.mouse_position[1]

    @property
    def cur_y(self):
        return self.mouse_position[0]

    @property
    def finish_cell(self):
        return self.finish[0]

    def reset_maze(self):
        self.maze = [[0 for _ in range(self.shape)] for _ in range(self.shape)]
        self.walls = [[0 for _ in range(self.shape)] for _ in range(self.shape)]

        for i in range(self.shape):
            self.walls[i][0] |= LEFT_WALL | LEFT_WALL_VISITED
            self.walls[i][self.shape - 1] |= RIGHT_WALL | RIGHT_WALL_VISITED
            self.walls[0][i] |= UP_WALL | UP_WALL_VISITED
            self.walls[self.shape - 1][i] |= DOWN_WALL | DOWN_WALL_VISITED

    def reset_position(self):
        self.mouse_position = self.start
        self.mouse_direction = self.start_direction
        requests.post(f'{self._api_route}/maze/restart?token={self._token}').json()

    def is_visited(self, position: Optional[tuple] = None):
        if position is None:
            position = self.mouse_position
        return (self.walls[position[0]][position[1]] & VISITED) == VISITED

    def set_walls(self, is_left_wall: bool, is_front_wall: bool, is_right_wall: bool):
        if self.is_visited():
            return
        left_wall_idx = (self.mouse_direction + 3) % 4
        front_wall_idx = self.mouse_direction
        right_wall_idx = (self.mouse_direction + 1) % 4

        _walls = [is_left_wall, is_front_wall, is_right_wall]
        _walls_directions = [left_wall_idx, front_wall_idx, right_wall_idx]

        for wall, wall_direction in zip(_walls, _walls_directions):
            neigh_y = (self.cur_y + self.NEIGHBOURS[wall_direction][0]) % self.shape
            neigh_x = (self.cur_x + self.NEIGHBOURS[wall_direction][1]) % self.shape
            neigh_wall_direction = (wall_direction + 2) % 4

            if wall and not (self.walls[self.cur_y][self.cur_x] & WALLS_VISITED[wall_direction]):
                self.walls[self.cur_y][self.cur_x] |= WALLS[wall_direction]
                self.walls[neigh_y][neigh_x] |= WALLS[neigh_wall_direction]

            self.walls[self.cur_y][self.cur_x] |= WALLS_VISITED[wall_direction]
            self.walls[neigh_y][neigh_x] |= WALLS_VISITED[neigh_wall_direction]

    def get_wall(self, direction: Directions) -> bool:
        _dir = (self.mouse_direction + direction) % 4
        return self.walls[self.cur_y][self.cur_x] & WALLS[_dir] == WALLS[_dir]

    def update_direction(self, change: Directions):
        self.mouse_direction = (self.mouse_direction + change) % 4

    def update_position(self):
        new_y = (self.cur_y + self.NEIGHBOURS[self.mouse_direction][0]) % self.shape
        new_x = (self.cur_x + self.NEIGHBOURS[self.mouse_direction][1]) % self.shape
        self.mouse_position = (new_y, new_x)

    def check_wall(self, position: tuple, wall: int):
        return (self.walls[position[0]][position[1]] & wall) == wall

    def get_neighbours(self, position: Optional[tuple] = None) -> list[dict]:
        if position is None:
            position = self.mouse_position
        neighbours = []
        for i in range(4):
            neigh_y = (position[0] + self.NEIGHBOURS[i][0]) % self.shape
            neigh_x = (position[1] + self.NEIGHBOURS[i][1]) % self.shape
            neigh_val = self.maze[neigh_y][neigh_x]
            if not self.check_wall(position, WALLS[i]):
                neigh_data = {
                    'dir': i,
                    'pos': (neigh_y, neigh_x),
                    'val': neigh_val
                }
                neighbours.append(neigh_data)
        return neighbours

    def floodfill(self):
        for y in range(self.shape):
            for x in range(self.shape):
                self.maze[y][x] = 0

        visited = [[0 for _ in range(self.shape)] for _ in range(self.shape)]

        to_process = queue.Queue()

        for cell in self.finish:
            visited[cell[0]][cell[1]] = 1
            to_process.put(cell)

        while not to_process.empty():
            current_pos = to_process.get()
            neighbours = self.get_neighbours(position=current_pos)
            for neigh in neighbours:
                neigh_y, neigh_x = neigh['pos']
                if not visited[neigh_y][neigh_x]:
                    visited[neigh_y][neigh_x] = 1
                    to_process.put((neigh_y, neigh_x))
                    self.maze[neigh_y][neigh_x] = self.maze[current_pos[0]][current_pos[1]] + 1

    def find_path(self, start: tuple) -> bool:
        self.path = []
        cur_val = self.maze[start[0]][start[1]]
        current_pos = start
        direction = self.mouse_direction

        while cur_val != 0:
            neighbours = self.get_neighbours(current_pos)
            for neigh in neighbours:
                neigh_y, neigh_x = neigh['pos']
                neigh_val = neigh['val']
                neigh_dir = neigh['dir']

                if (cur_val - 1) == neigh_val:
                    cur_val = neigh_val
                    current_pos = (neigh_y, neigh_x)
                    if direction == neigh_dir:
                        self.path.append('F')
                    elif (direction + 1) % 4 == neigh_dir:
                        self.path.append('R')
                        self.path.append('F')
                    elif (direction + 2) % 4 == neigh_dir:
                        self.path.append('A')
                        self.path.append('F')
                    elif (direction + 3) % 4 == neigh_dir:
                        self.path.append('L')
                        self.path.append('F')
                    direction = neigh_dir
                    break
        return len(self.path) > 0

    def get_next_move(self) -> str:
        return self.path.pop(0)

    def path_not_empty(self):
        return len(self.path) > 0

    def on_finish(self):
        return self.mouse_position in self.finish

    def lock_maze(self):
        for y in range(self.shape):
            for x in range(self.shape):
                for i, wall in enumerate(WALLS_VISITED):
                    if (self.walls[y][x] & wall) == 0:
                        self.walls[y][x] |= WALLS[i]

    def print_maze(self, show_weights: bool = True):
        if not DEBUG_LOGGING:
            return
        cell_shape = 3
        # print header
        for x in range(self.shape):
            print('+', end='')
            for sub_x in range(cell_shape):
                print('-', end='')
        print('+')
        for y in range(self.shape):
            for sub_y in range(cell_shape - 1):
                for x in range(self.shape):
                    if show_weights:
                        maze_value = self.maze[y][x]
                    else:
                        maze_value = int(self.is_visited((y, x)))
                    wall_value = self.walls[y][x]
                    left_wall = wall_value & LEFT_WALL
                    down_wall = wall_value & DOWN_WALL

                    if sub_y == 0:
                        if left_wall:
                            print('|', end='')
                        else:
                            print(' ', end='')
                        if y == self.cur_y and x == self.cur_x:
                            print(' ', end='')
                            print(DIRECTION_TO_CHAR[self.mouse_direction], end='')
                            print(' ', end='')
                        elif maze_value > 99:
                            print(maze_value, end='')
                        elif maze_value > 9:
                            print(' ', end='')
                            print(maze_value, end='')
                        else:
                            print(' ', end='')
                            print(maze_value, end='')
                            print(' ', end='')
                    else:
                        print('+', end='')
                        if down_wall:
                            print('---', end='')
                        else:
                            print('   ', end='')
                if sub_y == 1:
                    print('+')
                else:
                    print('|')


class Solver:
    def __init__(self, token: str):
        self.mouse = Mouse(token)
        self.maze = Maze(token)
        self.manual_control = True

    def _scan_position(self):
        self.mouse.update_sensor_data()
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)

    def _move_by_direction(self, direction: Directions):
        direction = direction % 4
        current_direction = self.maze.mouse_direction
        if current_direction == direction:
            self.mouse.pos -= CELL
            self.mouse.wait_until_position(CELL - SENSING_OFFSET)
        elif (current_direction + 1) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(Directions.right)
        elif (current_direction + 2) % 4 == direction:
            self.mouse.pos -= CELL
            self.mouse.wait_until_position(HALF_CELL - SENSING_OFFSET, stop_threshold=CENTER_REFERENCE)
            self.mouse.stop()
            self.mouse.around()
            self.maze.update_direction(Directions.down)
        elif (current_direction + 3) % 4 == direction:
            self.mouse.left()
            self.maze.update_direction(Directions.left)
        self.maze.update_position()

    def shortest(self):
        self.mouse.pos = CELL + HALF_CELL
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)
        self.maze.floodfill()
        if self.maze.get_wall(Directions.up):
            self.mouse.right_in_place()
            self.maze.update_direction(Directions.right)

        path_exists = self.maze.find_path(self.maze.mouse_position)
        self.maze.print_maze()
        recalculate = False

        if path_exists:
            while path_exists and not self.maze.on_finish():
                while self.maze.path_not_empty():
                    self._scan_position()
                    self.maze.print_maze()

                    next_path = self.maze.get_next_move()

                    if DEBUG_LOGGING:
                        print(f'Mouse position: {self.maze.mouse_position}')
                        print(f'Finish: {self.maze.finish_cell}')
                        print(f'Next move: {next_path}')
                        print(f'Path: {self.maze.path}')

                    if next_path == 'F':
                        if self.maze.get_wall(Directions.up):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction)
                    elif next_path == 'R':
                        if self.maze.get_wall(Directions.right):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + Directions.right)
                            self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'A':
                        self._move_by_direction(direction=self.maze.mouse_direction + Directions.down)
                        self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'L':
                        if self.maze.get_wall(Directions.left):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + Directions.left)
                            self.maze.get_next_move()  # pop F after turn

                    if recalculate:
                        self.maze.floodfill()
                        path_exists = self.maze.find_path(self.maze.mouse_position)
                        recalculate = False
                        self.maze.print_maze()
                        if DEBUG_LOGGING:
                            print("Recalculated!")
                        break

                    if self.maze.on_finish():
                        break

        self._scan_position()
        return path_exists

    def reset_position(self):
        self.maze.reset_position()
        self.mouse.update_sensor_data(init_update=True)


if __name__ == '__main__':
    time.sleep(10)
    solver = Solver(my_token)
    solver.shortest()
    # uncomment for local tests
    # input('Reset maze and press enter')
    solver.reset_position()
    time.sleep(10)
    solver.shortest()
    # input('Reset maze and press enter')
    solver.reset_position()
    time.sleep(10)
    solver.shortest()
