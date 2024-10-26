import queue

from src import constants


class Maze:
    shape = 16
    start = (15, 0)
    start_direction = constants.Directions.up
    finish = [(8, 8), (7, 7), (7, 8), (8, 7)]

    NEIGHBOURS = (
        (shape - 1, 0),
        (0, 1),
        (1, 0),
        (0, shape - 1),
    )

    def __init__(self):
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
        """
        clear maze weights and walls, set all boundary walls
        """
        self.maze = [[0 for _ in range(self.shape)] for _ in range(self.shape)]
        self.walls = [[0 for _ in range(self.shape)] for _ in range(self.shape)]

        for i in range(self.shape):
            self.walls[i][0] |= constants.LEFT_WALL | constants.LEFT_WALL_VISITED
            self.walls[i][self.shape - 1] |= constants.RIGHT_WALL | constants.RIGHT_WALL_VISITED
            self.walls[0][i] |= constants.UP_WALL | constants.UP_WALL_VISITED
            self.walls[self.shape - 1][i] |= constants.DOWN_WALL | constants.DOWN_WALL_VISITED

    def reset_position(self):
        """
        set mouse to initial position and direction, also reset simulator to initial position
        """
        self.mouse_position = self.start
        self.mouse_direction = self.start_direction

    def is_visited(self, position: tuple | None = None):
        """
        check if selected cell is visited. All 4 walls should be discovered
        :param position: requested cell to check. In case of no position current mouse position will be checked
        :return: True if cell is visited
        """
        if position is None:
            position = self.mouse_position
        return (self.walls[position[0]][position[1]] & constants.VISITED) == constants.VISITED

    def set_walls(self, is_left_wall: bool, is_front_wall: bool, is_right_wall: bool):
        """
        update wall data with data from mouse.
        :param is_left_wall: if mouse see left wall
        :param is_front_wall: if mouse see front wall
        :param is_right_wall: if mouse see right wall
        """
        if self.is_visited():
            return
        left_wall_idx = (self.mouse_direction + constants.Directions.left) % 4
        front_wall_idx = self.mouse_direction
        right_wall_idx = (self.mouse_direction + constants.Directions.right) % 4

        _walls = [is_left_wall, is_front_wall, is_right_wall]
        _walls_directions = [left_wall_idx, front_wall_idx, right_wall_idx]

        for wall, wall_direction in zip(_walls, _walls_directions):
            neigh_y = (self.cur_y + self.NEIGHBOURS[wall_direction][0]) % self.shape
            neigh_x = (self.cur_x + self.NEIGHBOURS[wall_direction][1]) % self.shape
            neigh_wall_direction = (wall_direction + 2) % 4

            if wall and not (self.walls[self.cur_y][self.cur_x] & constants.WALLS_VISITED[wall_direction]):
                self.walls[self.cur_y][self.cur_x] |= constants.WALLS[wall_direction]
                self.walls[neigh_y][neigh_x] |= constants.WALLS[neigh_wall_direction]

            self.walls[self.cur_y][self.cur_x] |= constants.WALLS_VISITED[wall_direction]
            self.walls[neigh_y][neigh_x] |= constants.WALLS_VISITED[neigh_wall_direction]

    def update_direction(self, change: constants.Directions):
        """
        change direction of mouse depend on current direction (only inside of maze class!)
        :param change: direction to change
        """
        self.mouse_direction = (self.mouse_direction + change) % 4

    def update_position(self):
        """
        move mouse one cell forward (only inside of maze class!)
        """
        new_y = (self.cur_y + self.NEIGHBOURS[self.mouse_direction][0]) % self.shape
        new_x = (self.cur_x + self.NEIGHBOURS[self.mouse_direction][1]) % self.shape
        self.mouse_position = (new_y, new_x)

    def check_wall(self, position: tuple, wall_idx: int):
        """
        check if requested cell contain wall from selected side
        :param position: cell to check
        :param wall_idx: wall index from WALLS
        :return: True if wall present
        """
        wall = constants.WALLS[wall_idx]
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
                neigh_data = {"dir": i, "pos": (neigh_y, neigh_x), "val": neigh_val}
                neighbours.append(neigh_data)
        return neighbours

    def floodfill(self):
        """
        Floodfill algorithm will fill weight map from finish to all cells. It will start with zero at finish and
        increase weights of all accessible neighbours to 1. With this map we can build shortest path by following
        minimum neighbour
        """
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
                neigh_y, neigh_x = neigh["pos"]
                if not visited[neigh_y][neigh_x]:
                    visited[neigh_y][neigh_x] = 1
                    to_process.put((neigh_y, neigh_x))
                    self.maze[neigh_y][neigh_x] = self.maze[current_pos[0]][current_pos[1]] + 1

    def find_path(self, start: tuple) -> bool:
        """
        This algorithm will try to find shortest path from requested position to finish using floodfilled weight map.
        Algorithm just follow minimum neighbour.
        :param start: position of the start
        :return: True if path to finish exist
        """
        self.path = []
        cur_val = self.maze[start[0]][start[1]]
        current_pos = start
        direction = self.mouse_direction

        while cur_val != 0:
            neighbours = self.get_neighbours(current_pos)
            for neigh in neighbours:
                neigh_y, neigh_x = neigh["pos"]
                neigh_val = neigh["val"]
                neigh_dir = neigh["dir"]

                if (cur_val - 1) == neigh_val:
                    cur_val = neigh_val
                    current_pos = (neigh_y, neigh_x)
                    if direction == neigh_dir:
                        self.path.append("F")
                    elif (direction + 1) % 4 == neigh_dir:
                        self.path.append("R")
                        self.path.append("F")
                    elif (direction + 2) % 4 == neigh_dir:
                        self.path.append("A")
                        self.path.append("F")
                    elif (direction + 3) % 4 == neigh_dir:
                        self.path.append("L")
                        self.path.append("F")
                    direction = neigh_dir
                    break
        return len(self.path) > 0

    def get_next_move(self) -> str:
        """
        take first movement from path string
        :return:
        """
        return self.path.pop(0)

    def path_not_empty(self) -> bool:
        """
        check if we have movements to go
        """
        return len(self.path) > 0

    def on_finish(self) -> bool:
        """
        check if mouse already on finish
        :return:
        """
        return self.mouse_position in self.finish

    def lock_maze(self):
        """
        set all not visited walls to 1, so robot can go only by explored cells
        """
        for y in range(self.shape):
            for x in range(self.shape):
                for i, wall in enumerate(constants.WALLS_VISITED):
                    if (self.walls[y][x] & wall) == 0:
                        self.walls[y][x] |= constants.WALLS[i]

    def __str__(self):
        maze = []
        if not constants.DEBUG_LOGGING:
            return
        cell_shape = 3
        # print header
        for x in range(self.shape):
            maze.append('+')
            for sub_x in range(cell_shape):
                maze.append('-')
        maze.append('+\n')
        for y in range(self.shape):
            for sub_y in range(cell_shape - 1):
                for x in range(self.shape):
                    maze_value = str(self.maze[y][x])
                    value_length = len(maze_value)
                    wall_value = self.walls[y][x]
                    left_wall = wall_value & constants.LEFT_WALL
                    down_wall = wall_value & constants.DOWN_WALL

                    if sub_y == 0:
                        if left_wall:
                            maze.append('|')
                        else:
                            maze.append(' ')
                        if y == self.cur_y and x == self.cur_x:
                            maze.append(' ')
                            maze.append(constants.DIRECTION_TO_CHAR[self.mouse_direction])
                            maze.append(' ')
                        elif value_length > 2:
                            maze.append(maze_value)
                        elif value_length > 1:
                            maze.append(' ')
                            maze.append(maze_value)
                        else:
                            maze.append(' ')
                            maze.append(maze_value)
                            maze.append(' ')
                    else:
                        maze.append('+')
                        if down_wall:
                            maze.append('---')
                        else:
                            maze.append('   ')
                if sub_y == 1:
                    maze.append('+\n')
                else:
                    maze.append('|\n')
        return ''.join(maze)
