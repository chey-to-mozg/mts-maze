import math
import time
from enum import Enum

import requests

from src import constants


class Sensors(str, Enum):
    left = '2'
    left_45 = '6'
    up = '4'
    right = '5'
    right_45 = '3'
    down = '1'


class Mouse:
    def __init__(self, sim: bool = False):
        super().__init__()
        self._token = ""
        self._api_route = "http://127.0.0.1:8801/api/v1" if sim else constants.MOUSE_IP
        self._sim = sim

        self.left_wall_distance = 0
        self.front_wall_distance = 0
        self.right_wall_distance = 0
        self.left_wall = False
        self.front_wall = False
        self.right_wall = False
        self._request_delay = 1  # secs

        self.angle = 0
        self.reference_angle = 0
        self.angle_error = 0
        self.pos = 0
        self.speed = 0
        self.max_speed = constants.MAX_SPEED

        self.rot_error = 0
        self._steering_enabled = True

        self.update_sensor_data(init_update=True)

    # api control
    def _make_action(self, action: str, distance: int):
        """
        basic movement control
        :param action: string from corresponding api call (forward | left | right)
        :param distance: distancve for forward and angle for left\right
        """
        if self._sim:
            requests.post(f'{self._api_route}/robot-cells/{action}?token={self._token}')
            time.sleep(0.1)
        else:
            body = {'direction': action, 'id': constants.MOUSE_ID, 'len': distance}
            requests.put(f"{self._api_route}/move", json=body)
            time.sleep(self._request_delay)

    def forward(self, distance: int = constants.CELL):
        self._make_action("forward", distance)

    def right(self, angle: int = constants.TURN_90):
        # set reference to be always "right" number i.e. 0, 45, 90, 135 e.t.c
        self.reference_angle = (self.reference_angle + angle) % 360
        # calculate diff between current angle and desired. Add offset to calibrate angle
        diff = (self.reference_angle - self.angle + constants.ANGLE_OFFSET) % 360
        self._make_action("right", diff)

    def left(self, angle: int = constants.TURN_90):
        # set reference to be always "right" number i.e. 0, 45, 90, 135 e.t.c
        self.reference_angle = (self.reference_angle - angle) % 360
        # calculate diff between current angle and desired. Add offset to calibrate angle
        diff = (self.angle - self.reference_angle + constants.ANGLE_OFFSET) % 360
        self._make_action("left", diff)

    def backward(self, distance: int = 100):
        self._make_action("backward", distance)

    def calibrate_back_wall(self):
        if self._sim:
            return
        self.backward()
        self.forward(distance=constants.TO_CENTER)

    def around(self):
        self.left()
        self.left()

    def set_delay(self, delay: float):
        self._request_delay = delay

    # manual control # move to separate class?

    def _move_motors(self, pwm_left: int | float, pwm_right: int | float, m_time: int = 100):
        """
        run motors with requested pwm and time
        :param pwm_left:  value for left motor
        :param pwm_right: value for right motor
        :param m_time:    execution time
        """
        pwm_left = int(pwm_left)
        pwm_right = int(pwm_right)
        body = {'id': constants.MOUSE_ID, 'l': pwm_left, 'r': pwm_right, 'l_time': m_time, 'r_time': m_time}
        requests.put(f"{self._api_route}/motor", json=body)
        # TODO check request duration and calculate actual delay
        # time.sleep(m_time)

    def _move(self, dist: float, stop_threshold: float):
        """
        go forward until dist reached
        :param dist: distance to go
        :param stop_threshold: front sensor threshold to stop action
        """
        self.update_sensor_data()
        while self.pos < dist:
            if self.speed < self.max_speed:
                left_pwm = constants.FORWARD_SPEED + self.rot_error
                right_pwm = constants.FORWARD_SPEED - self.rot_error
                self._move_motors(left_pwm, right_pwm)
            else:
                time.sleep(0.1)
            self.update_sensor_data()
            if self.front_wall_distance < stop_threshold:
                return

    def move(self, dist: float, stop_threshold: float = constants.PRE_TURN_THRESHOLD):
        """
        Move requested dist
        :param dist: distance to go
        :param stop_threshold: front sensor threshold to stop action
        :return:
        """
        self.pos = 0
        self._move(dist, stop_threshold)

    def wait_until_position(self, dist: float, stop_threshold: float = constants.PRE_TURN_THRESHOLD):
        """
        move mouse without resetting the distance already reached
        :param dist: distance to wait for
        :param stop_threshold: front sensor threshold to stop action
        """
        self._move(dist, stop_threshold)

    def stop(self):
        """
        call for stop motors until speed is zero
        """
        self.update_sensor_data()
        while self.speed > 5:
            self._move_motors(-constants.FORWARD_SPEED, -constants.FORWARD_SPEED)
            self.update_sensor_data()

    def right_in_place(self):
        """
        turn mouse in place right using gyro
        """
        self._steering_enabled = False
        self.reference_angle += 90
        self.update_sensor_data()
        while self.angle < self.reference_angle - constants.ANGLE_OFFSET:
            self._move_motors(constants.ROTATION_SPEED, -constants.ROTATION_SPEED)
            self.update_sensor_data()

        self._steering_enabled = True

    def left_in_place(self):
        """
        turn mouse in place left using gyro
        """
        self._steering_enabled = False
        self.reference_angle -= 90
        self.update_sensor_data()
        while self.angle > self.reference_angle + constants.ANGLE_OFFSET:
            self._move_motors(-constants.ROTATION_SPEED, constants.ROTATION_SPEED)
            self.update_sensor_data()

        self._steering_enabled = True

    def around_in_place(self):
        """
        turn around using 2 in place turns
        """
        self.left_in_place()
        self.left_in_place()

    def get_sensors(self) -> dict:
        """
        get sensor data. contains different values from all sensors
        :return: dict with sensors data
        """
        # data example:
        # return {
        #     'laser': {
        #         '1': 239, '2': 59, '3': 82, '4': 225, '5': 70, '6': 93
        #     },
        #     'imu': {
        #         'roll': -1, 'pitch': 0, 'yaw': 121
        #     }
        # }
        if self._sim:
            return requests.get(f'{self._api_route}/robot-cells/sensor-data?token={self._token}').json()
        else:
            body = {'id': constants.MOUSE_ID, 'type': 'all'}
            return requests.post(f"{self._api_route}/sensor", json=body).json()

    def _calc_errors(self):
        """
        Calculate wall and gyro errors
        """
        ang_error = (self.reference_angle - self.angle) * constants.KP_ROT

        pos_error = 0
        left_err = self.left_wall_distance - constants.LEFT_REFERENCE
        right_err = self.right_wall_distance - constants.RIGHT_REFERENCE
        if self.left_wall and self.right_wall:
            pos_error = left_err - right_err
        elif self.left_wall:
            pos_error = 2 * left_err
        elif self.right_wall:
            pos_error = 2 * right_err
        pos_error *= constants.KP_STEERING

        if not self._steering_enabled:
            pos_error = 0

        self.rot_error = ang_error - pos_error

    def _calc_dist(self) -> float:
        """
        calculate distance between data updates
        :return: reached distance
        """
        return math.sqrt((self.prev_y - self.offset_y) ** 2 + (self.prev_x - self.offset_x) ** 2)

    def _update_movement(self):
        """
        update movement related data i.e. position, angle, speed
        """
        diff = self._calc_dist()
        self.speed = diff / 0.1
        self.pos += diff
        self.prev_y = self.offset_y
        self.prev_x = self.offset_x

    def update_sensor_data(self, init_update: bool = False):
        """
        read sensor data and update wall related data
        """
        sensor_data = self.get_sensors()
        if self._sim:
            self.left_wall_distance = sensor_data['left_side_distance']
            self.front_wall_distance = sensor_data['front_distance']
            self.right_wall_distance = sensor_data['right_side_distance']
            self.left_wall = self.left_wall_distance < constants.WALL_THRESHOLD
            self.front_wall = self.front_wall_distance < constants.FRONT_WALL_THRESHOLD
            self.right_wall = self.right_wall_distance < constants.WALL_THRESHOLD
            self.angle = sensor_data['rotation_yaw']
            if self.angle < 0:
                self.angle += 360
        else:
            lasers = sensor_data["laser"]
            self.front_wall_distance = lasers[Sensors.up]
            self.left_wall_distance = lasers[Sensors.left]
            self.right_wall_distance = lasers[Sensors.right]
            self.left_wall = self.left_wall_distance < constants.WALL_THRESHOLD
            self.front_wall = self.front_wall_distance < constants.FRONT_WALL_THRESHOLD
            self.right_wall = self.right_wall_distance < constants.WALL_THRESHOLD

            imu = sensor_data['imu']
            self.angle = imu["yaw"] - self.angle_error
            if self.angle < 0:
                self.angle += 360

            if init_update:
                self.angle_error = self.angle
                self.angle = 0
        # self._update_movement()
        #
        # self._calc_errors()

    def __str__(self):
        return (
            f"< {self.left_wall_distance} {self.left_wall} \n"
            f"^ {self.front_wall_distance} {self.front_wall} \n"
            f"> {self.right_wall_distance} {self.right_wall} \n"
            f"angle: {self.angle} ({self.reference_angle})\n"
            f"error: {self.rot_error}"
        )


def print_sensor_data():
    mouse = Mouse()
    while True:
        mouse.update_sensor_data()
        print(mouse)
        time.sleep(0.1)


def check_response_time():
    mouse = Mouse()

    mouse.update_sensor_data()
    print(mouse)
    mouse.right()
    mouse.update_sensor_data()
    print(mouse)
    mouse.forward()
    mouse.update_sensor_data()
    print(mouse)
    mouse.forward()
    mouse.update_sensor_data()
    print(mouse)
    mouse.left()
    mouse.update_sensor_data()
    print(mouse)


def check_turns():
    mouse = Mouse()

    mouse.update_sensor_data()
    print(mouse)
    mouse.right()
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')
    mouse.right()
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')
    mouse.left(angle=180)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')

    mouse.update_sensor_data()
    print(mouse)
    mouse.left()
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')
    mouse.left()
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')
    mouse.right(angle=180)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'{mouse.reference_angle}, real={mouse.angle} diff={mouse.angle - angle}')


def check_45_turn():
    mouse = Mouse()

    mouse.update_sensor_data()
    print(mouse)
    mouse.right(angle=constants.TURN_45)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'45, diff={mouse.angle - angle}')
    print(mouse)
    mouse.right(angle=constants.TURN_45)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'90, diff={mouse.angle - angle}')
    print(mouse)

    mouse.left(angle=constants.TURN_45)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'45, diff={mouse.angle - angle}')
    print(mouse)
    mouse.left(angle=constants.TURN_45)
    angle = mouse.angle
    mouse.update_sensor_data()
    print(f'90, diff={mouse.angle - angle}')
    print(mouse)


def check_pre_turn_with_45():
    mouse = Mouse()

    mouse.right()
    mouse.forward()
    mouse.forward(distance=constants.HALF_CELL)
    mouse.left(angle=constants.TURN_45)
    mouse.forward(distance=constants.DIAG_CELL)
    mouse.left(angle=constants.TURN_45)
    mouse.forward(distance=constants.HALF_CELL)


def check_to_center_calibration():
    mouse = Mouse()

    mouse.update_sensor_data()
    print(mouse)

    mouse.backward()
    mouse.update_sensor_data()
    print(mouse)

    mouse.forward(distance=constants.TO_CENTER)
    mouse.update_sensor_data()
    print(mouse)


if __name__ == "__main__":
    # print_sensor_data()
    # check_response_time()
    # check_turns()
    # check_45_turn()
    check_to_center_calibration()
