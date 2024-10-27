from src import constants
from src.Maze import Maze
from src.Mouse import Mouse


class Solver:
    path_to_dir = {
        'F': constants.Directions.up,
        'R': constants.Directions.right,
        'A': constants.Directions.down,
        'L': constants.Directions.left,
    }

    def __init__(self, sim: bool = False, load_maze: bool = False, calibrate_back_wall: bool = False):
        self.mouse = Mouse(sim)
        self.maze = Maze(load_maze)
        self.calibrate_back_wall = calibrate_back_wall

    def _scan_position(self):
        """
        get values from mouse sensors and update wall information inside maze
        """
        self.mouse.update_sensor_data()
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)

    def _move_by_direction(self, direction: constants.Directions):
        """
        move mouse 1 cell by chosen direction relative to mouse
        :param direction: direction to move
        """
        direction = direction % 4
        current_direction = self.maze.mouse_direction
        if current_direction == direction:
            self.mouse.forward()
            self.maze.update_position()
        elif (current_direction + constants.Directions.right) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(constants.Directions.right)
        elif (current_direction + constants.Directions.down) % 4 == direction:
            self.mouse.around()
            self.maze.update_direction(constants.Directions.down)
        elif (current_direction + constants.Directions.left) % 4 == direction:
            self.mouse.left()
            self.maze.update_direction(constants.Directions.left)

    def _make_mouse_move(self, change: constants.Directions) -> bool:
        maze_direction = self.maze.mouse_direction + change
        moved = False
        if not self.maze.check_wall(self.maze.mouse_position, maze_direction):
            self._move_by_direction(direction=maze_direction)
            back_wall_exists = self.maze.check_wall(
                self.maze.mouse_position, maze_direction + constants.Directions.down
            )
            if self.calibrate_back_wall and back_wall_exists:
                self.mouse.calibrate_back_wall()
            moved = True
        return moved

    def shortest(self):
        """
        follow shortest path after floodfill. In case we cant move to desired direction because of wall we will rebuild
        weight map to find new path
        :return:
        """
        self.maze.floodfill()
        path_exists = self.maze.find_path(self.maze.mouse_position)
        if constants.DEBUG_LOGGING:
            print(self.maze)
        recalculate = False

        if path_exists:
            while path_exists and not self.maze.on_finish():
                while self.maze.path_not_empty():
                    self._scan_position()

                    next_path = self.maze.get_next_move()
                    mouse_pos = self.maze.mouse_position
                    if constants.DEBUG_LOGGING:
                        print(self.maze)
                        print(f'Mouse position: {mouse_pos}')
                        print(f'Finish: {self.maze.finish}')
                        print(f'Next move: {next_path}')
                        print(f'Path: {self.maze.path}')
                        print(self.mouse)

                    if not self._make_mouse_move(self.path_to_dir[next_path]):
                        recalculate = True

                    if recalculate:
                        self.maze.floodfill()
                        path_exists = self.maze.find_path(mouse_pos)
                        recalculate = False
                        if constants.DEBUG_LOGGING:
                            print("Recalculated!")
                        break

                    if self.maze.on_finish():
                        break

        self._scan_position()
        return path_exists

    def reset_position(self):
        """
        set mouse position inside this class to initial place
        """
        self.maze.reset_position()
        self.mouse.update_sensor_data(init_update=True)


if __name__ == '__main__':
    solver = Solver(sim=True, load_maze=False, calibrate_back_wall=True)
    solver.shortest()
    solver.maze.save_maze()
    input('Move robot to the start')
    solver.mouse.set_delay(0.5)
    solver.reset_position()
    solver.shortest()
    input('Move robot to the start')
    solver.calibrate_back_wall = False
    solver.reset_position()
    solver.shortest()
