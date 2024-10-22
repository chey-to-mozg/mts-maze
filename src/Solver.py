from src import constants
from src.Maze import Maze
from src.Mouse import Mouse


class Solver:
    def __init__(self):
        self.mouse = Mouse()
        self.maze = Maze()

    def _scan_position(self):
        """
        get values from mouse sensors and update wall information inside maze
        """
        self.mouse.update_sensor_data()
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)

    # def _move_by_direction(self, direction: constants.Directions):
    #     """
    #     move mouse 1 cell by chosen direction relative to maze
    #     :param direction: direction to move
    #     """
    #     direction = direction % 4
    #     current_direction = self.maze.mouse_direction
    #     if current_direction == direction:
    #         self.mouse.pos -= constants.CELL
    #         self.mouse.wait_until_position(constants.CELL - constants.SENSING_OFFSET)
    #     elif (current_direction + 1) % 4 == direction:
    #         self.mouse.right()
    #         self.maze.update_direction(constants.Directions.right)
    #     elif (current_direction + 2) % 4 == direction:
    #         self.mouse.pos -= constants.CELL
    #         self.mouse.wait_until_position(
    #             constants.HALF_CELL - constants.SENSING_OFFSET, stop_threshold=constants.CENTER_REFERENCE
    #         )
    #         self.mouse.stop()
    #         self.mouse.around()
    #         self.maze.update_direction(constants.Directions.down)
    #     elif (current_direction + 3) % 4 == direction:
    #         self.mouse.left()
    #         self.maze.update_direction(constants.Directions.left)
    #     self.maze.update_position()
    def _move_by_direction(self, direction: constants.Directions):
        """
        move mouse 1 cell by chosen direction relative to maze
        :param direction: direction to move
        """
        direction = direction % 4
        current_direction = self.maze.mouse_direction
        if current_direction == direction:
            self.mouse.forward()
        elif (current_direction + 1) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(constants.Directions.right)
        elif (current_direction + 2) % 4 == direction:
            self.mouse.right()
            self.maze.update_direction(constants.Directions.down)
        elif (current_direction + 3) % 4 == direction:
            self.mouse.left()
            self.maze.update_direction(constants.Directions.left)
        self.maze.update_position()

    def shortest(self):
        """
        follow shortest path after floodfill. In case we cant move to desired direction because of wall we will rebuild
        weight map to find new path
        :return:
        """
        self.mouse.pos = constants.CELL + constants.HALF_CELL
        self.maze.set_walls(self.mouse.left_wall, self.mouse.front_wall, self.mouse.right_wall)
        self.maze.floodfill()
        if self.maze.check_wall(self.maze.mouse_position, constants.Directions.up):
            self.mouse.right_in_place()
            self.maze.update_direction(constants.Directions.right)

        path_exists = self.maze.find_path(self.maze.mouse_position)
        self.maze.print_maze()
        recalculate = False

        if path_exists:
            while path_exists and not self.maze.on_finish():
                while self.maze.path_not_empty():
                    self._scan_position()
                    self.maze.print_maze()

                    next_path = self.maze.get_next_move()

                    if constants.DEBUG_LOGGING:
                        print(f'Mouse position: {self.maze.mouse_position}')
                        print(f'Finish: {self.maze.finish_cell}')
                        print(f'Next move: {next_path}')
                        print(f'Path: {self.maze.path}')

                    if next_path == 'F':
                        if self.maze.check_wall(self.maze.mouse_position, constants.Directions.up):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction)
                    elif next_path == 'R':
                        if self.maze.check_wall(self.maze.mouse_position, constants.Directions.right):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.right)
                            self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'A':
                        self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.down)
                        self.maze.get_next_move()  # pop F after turn
                    elif next_path == 'L':
                        if self.maze.check_wall(self.maze.mouse_position, constants.Directions.left):
                            recalculate = True
                        else:
                            self._move_by_direction(direction=self.maze.mouse_direction + constants.Directions.left)
                            self.maze.get_next_move()  # pop F after turn

                    if recalculate:
                        self.maze.floodfill()
                        path_exists = self.maze.find_path(self.maze.mouse_position)
                        recalculate = False
                        self.maze.print_maze()
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
    solver = Solver()
    solver.shortest()
