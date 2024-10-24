from src import constants
from src.Maze import Maze
from src.Mouse import Mouse


class Solver:
    def __init__(self, sim: bool = False):
        self.mouse = Mouse(sim)
        self.maze = Maze()

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
            pass
        elif (current_direction + constants.Directions.right) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(constants.Directions.right)
        elif (current_direction + constants.Directions.down) % 4 == direction:
            self.mouse.around()
            self.maze.update_direction(constants.Directions.down)
        elif (current_direction + constants.Directions.left) % 4 == direction:
            self.mouse.left()
            self.maze.update_direction(constants.Directions.left)
        self.mouse.forward()
        self.maze.update_position()

    def shortest(self):
        """
        follow shortest path after floodfill. In case we cant move to desired direction because of wall we will rebuild
        weight map to find new path
        :return:
        """
        self.maze.floodfill()
        path_exists = self.maze.find_path(self.maze.mouse_position)
        print(self.maze)
        recalculate = False

        if path_exists:
            while path_exists and not self.maze.on_finish():
                while self.maze.path_not_empty():
                    self._scan_position()
                    print(self.maze)

                    next_path = self.maze.get_next_move()

                    if constants.DEBUG_LOGGING:
                        print(f'Mouse position: {self.maze.mouse_position}')
                        print(f'Finish: {self.maze.finish}')
                        print(f'Next move: {next_path}')
                        print(f'Path: {self.maze.path}')
                        print(self.mouse)

                    if next_path == 'F':
                        if self.mouse.front_wall:
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction)
                    elif next_path == 'R':
                        if self.mouse.right_wall:
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.right)
                            self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'A':
                        self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.down)
                        self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'L':
                        if self.mouse.left_wall:
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.left)
                            self.maze.get_next_move()  # pop F after turn

                    if recalculate:
                        self.maze.floodfill()
                        path_exists = self.maze.find_path(self.maze.mouse_position)
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
    solver = Solver(sim=True)
    solver.shortest()
    input('Move robot to the start')
    solver.reset_position()
    solver.shortest()
    input('Move robot to the start')
    solver.reset_position()
    solver.shortest()
