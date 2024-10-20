import requests
from enum import IntEnum
import time

my_token = 'token'
DEBUG_LOGGING = True


class Mouse:
    _wall_threshold = 100

    def __init__(self, token: str):
        self._api_route = 'http://127.0.0.1:8801/api/v1'
        self._token = token
        self.left_wall_distance = 0
        self.front_wall_distance = 0
        self.right_wall_distance = 0
        self.left_wall = False
        self.front_wall = False
        self.right_wall = False

    def _make_action(self, action: str):
        """
        basic movement control
        :param action: string from corresponding api call (forward | left | right)
        """
        requests.post(f'{self._api_route}/robot-cells/{action}?token={self._token}')
        time.sleep(0.1)

    def forward(self):
        self._make_action('forward')

    def left(self):
        self._make_action('left')

    def right(self):
        self._make_action('right')

    def around(self):
        self.left()
        self.left()

    def get_sensors(self) -> dict:
        """
        get sensor data. contains different values from all sensors
        :return: dict with sensors data
        """
        return requests.get(f'{self._api_route}/robot-cells/sensor-data?token={self._token}').json()

    def update_walls(self):
        """
        read sensor data and update wall related data
        """
        sensor_data = self.get_sensors()
        self.left_wall_distance = sensor_data['left_side_distance']
        self.front_wall_distance = sensor_data['front_distance']
        self.right_wall_distance = sensor_data['right_side_distance']
        self.left_wall = self.left_wall_distance < self._wall_threshold
        self.front_wall = self.front_wall_distance < self._wall_threshold
        self.right_wall = self.right_wall_distance < self._wall_threshold


class Directions(IntEnum):
    up = 0
    right = 1
    down = 2
    left = 3

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

WALLS_VISITED = (UP_WALL_VISITED, RIGHT_WALL_VISITED, DOWN_WALL_VISITED, LEFT_WALL_VISITED)
WALLS = (UP_WALL, RIGHT_WALL, DOWN_WALL, LEFT_WALL)
DIRECTION_TO_CHAR = ('^', '>', 'v', '<')

# just mapping of presented walls to corresponding number
RESULT_WALLS_MAPPING = {
    0: 0,
    LEFT_WALL: 1,
    UP_WALL: 2,
    RIGHT_WALL: 3,
    DOWN_WALL: 4,
    LEFT_WALL | DOWN_WALL: 5,
    RIGHT_WALL | DOWN_WALL: 6,
    UP_WALL | RIGHT_WALL: 7,
    UP_WALL | LEFT_WALL: 8,
    LEFT_WALL | RIGHT_WALL: 9,
    UP_WALL | DOWN_WALL: 10,
    UP_WALL | RIGHT_WALL | DOWN_WALL: 11,
    LEFT_WALL | UP_WALL | RIGHT_WALL: 12,
    UP_WALL | DOWN_WALL | LEFT_WALL: 13,
    RIGHT_WALL | DOWN_WALL | LEFT_WALL: 14,
    UP_WALL | RIGHT_WALL | DOWN_WALL | LEFT_WALL: 15,
}


class Maze:
    shape = 16
    start = (15, 0)
    start_direction = Directions.up
    finish = (8, 8)

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

    def reset_maze(self):
        """
        clear maze weights and walls, set all boundary walls
        """
        self.maze = [[0 for _ in range(self.shape)] for _ in range(self.shape)]
        self.walls = [[0 for _ in range(self.shape)] for _ in range(self.shape)]

        for i in range(self.shape):
            self.walls[i][0] |= LEFT_WALL | LEFT_WALL_VISITED
            self.walls[i][self.shape - 1] |= RIGHT_WALL | RIGHT_WALL_VISITED
            self.walls[0][i] |= UP_WALL | UP_WALL_VISITED
            self.walls[self.shape - 1][i] |= DOWN_WALL | DOWN_WALL_VISITED

    def reset_position(self):
        """
        set mouse to initial position and direction, also reset simulator to initial position
        """
        self.mouse_position = self.start
        self.mouse_direction = self.start_direction
        requests.post(f'{self._api_route}/maze/restart?token={self._token}').json()

    def is_visited(self, position: tuple | None = None) -> bool:
        """
        check if selected cell is visited. All 4 walls should be discovered
        :param position: requested cell to check. In case of no position current mouse position will be checked
        :return: True if cell is visited
        """
        if position is None:
            position = self.mouse_position
        return (self.walls[position[0]][position[1]] & VISITED) == VISITED

    def set_walls(self, is_left_wall: bool, is_front_wall: bool, is_right_wall: bool):
        """
        update wall data with data from mouse.
        :param is_left_wall: if mouse see left wall
        :param is_front_wall: if mouse see front wall
        :param is_right_wall: if mouse see right wall
        :return:
        """
        if self.is_visited():
            return
        left_wall_idx = (self.mouse_direction + 3) % 4
        front_wall_idx = self.mouse_direction
        right_wall_idx = (self.mouse_direction + 1) % 4

        _walls = [is_left_wall, is_front_wall, is_right_wall]
        _walls_idx = [left_wall_idx, front_wall_idx, right_wall_idx]

        for i in range(len(_walls)):
            wall_idx = _walls_idx[i]
            neigh_y = (self.cur_y + self.NEIGHBOURS[wall_idx][0]) % self.shape
            neigh_x = (self.cur_x + self.NEIGHBOURS[wall_idx][1]) % self.shape
            neigh_wall_idx = (wall_idx + 2) % 4

            if _walls[i] and not (self.walls[self.cur_y][self.cur_x] & WALLS_VISITED[wall_idx]):
                self.walls[self.cur_y][self.cur_x] |= WALLS[wall_idx]
                self.walls[neigh_y][neigh_x] |= WALLS[neigh_wall_idx]

            self.walls[self.cur_y][self.cur_x] |= WALLS_VISITED[wall_idx]
            self.walls[neigh_y][neigh_x] |= WALLS_VISITED[neigh_wall_idx]

    def update_direction(self, change: Directions):
        """
        change direction of mouse depend on current direction (only inside of maze class!)
        :param change:  0 -- do not change direction
                        1 -- turn mouse right
                        2 -- turn mouse around
                        3 -- turn mouse left
        """
        self.mouse_direction = (self.mouse_direction + change) % 4

    def update_position(self):
        """
        move mouse one cell forward (only inside of maze class!)
        """
        new_y = (self.cur_y + self.NEIGHBOURS[self.mouse_direction][0]) % self.shape
        new_x = (self.cur_x + self.NEIGHBOURS[self.mouse_direction][1]) % self.shape
        self.mouse_position = (new_y, new_x)

    def check_wall(self, position: tuple, wall_idx: int) -> bool:
        """
        check if requested cell contain wall from selected side
        :param position: cell to check
        :param wall_idx: wall index from WALLS
        :return: True if wall present
        """
        wall = WALLS[wall_idx]
        return (self.walls[position[0]][position[1]] & wall) == wall

    def get_neighbours(self, position: tuple | None = None) -> list[dict]:
        """
        Get all accessible neighbours from requested cell.
        :param position: cell to check. In case of no value will check mouse cell
        :return: list with neighbours data
        """
        if position is None:
            position = self.mouse_position
        neighbours = []
        for i in range(4):
            neigh_y = (position[0] + self.NEIGHBOURS[i][0]) % self.shape
            neigh_x = (position[1] + self.NEIGHBOURS[i][1]) % self.shape
            neigh_val = self.maze[neigh_y][neigh_x]
            if not self.check_wall(position, i):
                neigh_data = {
                    'dir': i,
                    'pos': (neigh_y, neigh_x),
                    'val': neigh_val
                }
                neighbours.append(neigh_data)
        return neighbours

    def lock_maze(self):
        """
        set all not visited walls to 1, so robot can go only by explored cells
        """
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

    def post_result(self):
        """
        Function to post results for first task
        :return: obtained score
        """
        result = [[0 for _ in range(self.shape)] for _ in range(self.shape)]
        mask = UP_WALL | RIGHT_WALL | DOWN_WALL | LEFT_WALL
        for y in range(self.shape):
            for x in range(self.shape):
                result[y][x] = RESULT_WALLS_MAPPING[self.walls[y][x] & mask]
        return requests.post(f'{self._api_route}/matrix/send?token={self._token}', json=result).json()


class Solver:
    def __init__(self, token: str):
        self.mouse = Mouse(token)
        self.maze = Maze(token)

    def _scan_position(self):
        """
        get values from mouse sensors and update wall information inside maze
        """
        self.mouse.update_walls()
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)

    def _move_by_direction(self, direction: Directions):
        """
        move mouse 1 cell by chosen direction relative to maze
        :param direction: direction to move
        """
        direction = direction % 4
        current_direction = self.maze.mouse_direction
        if current_direction == direction:
            pass
        elif (current_direction + 1) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(Directions.right)
        elif (current_direction + 2) % 4 == direction:
            self.mouse.around()
            self.maze.update_direction(Directions.down)
        elif (current_direction + 3) % 4 == direction:
            self.mouse.left()
            self.maze.update_direction(Directions.left)
        self.mouse.forward()
        self.maze.update_position()

    def left_hand(self):
        # TODO try implement this algo first
        pass

    def reset_position(self):
        self.maze.reset_position()

    def post_task_1(self):
        return self.maze.post_result()


if __name__ == '__main__':
    solver = Solver(my_token)
    solver.left_hand()
    print(solver.post_task_1())

