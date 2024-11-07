import time
from enum import Enum

import requests

from src import constants
from src.utils import debug


class Sensors(str, Enum):
    left = 'left'
    left_45 = 'left45'
    up = 'forward'
    right = 'right'
    right_45 = 'right45'
    down = 'backward'


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

        self.angle = 0
        self.reference_angle = 0
        self.angle_error = 0
        self.prev_angle = 0
        self.pos = 0
        self.speed = 0
        self.max_speed = constants.MAX_SPEED

        self.rot_error = 0
        self._steering_enabled = False

        self.backward()
        self.update_sensor_data(init_update=True)
        self.forward(constants.TO_CENTER)

    def _move_motors(self, pwm_left: int | float, pwm_right: int | float, m_time: int = 50):
        """
        run motors with requested pwm and time
        :param pwm_left:  value for left motor
        :param pwm_right: value for right motor
        :param m_time:    execution time
        """
        pwm_left = int(pwm_left)
        pwm_right = int(pwm_right)
        body = {
            'id': constants.MOUSE_ID,
            'l': pwm_left,
            'r': pwm_right,
            'l_time': m_time,
            'r_time': m_time,
        }
        requests.put(f"{self._api_route}/motor", json=body)
        # TODO check request duration and calculate actual delay
        # time.sleep(m_time)

    def _move(self, distance: int, stop_threshold: float):
        """
        go forward until dist reached
        :param distance: distance to go
        :param stop_threshold: front sensor threshold to stop action
        """
        pwm = constants.FORWARD_SPEED
        self.update_sensor_data()
        debug(f'forward {self.pos} -> {distance}')
        while self.pos < distance:
            pwm_left = pwm + self.rot_error
            pwm_right = pwm - self.rot_error
            debug(f'{pwm_left} {pwm_right}')
            self._move_motors(pwm_left, pwm_right)
            self.update_sensor_data()
            # TODO add front wall check

    def forward(self, distance: int = constants.CELL, stop_threshold: float = constants.PRE_TURN_THRESHOLD):
        """
        Move requested dist
        :param distance: distance to go
        :param stop_threshold: front sensor threshold to stop action
        :return:
        """
        self.pos = 0
        self._move(distance, stop_threshold)

    def backward(self, distance: int = 100):
        self.pos = 0
        pwm = constants.FORWARD_SPEED
        while abs(self.pos) < distance:  # check encoders sign!
            self._move_motors(-pwm, -pwm)
            self.update_sensor_data()

    def calibrate_back_wall(self):
        if self._sim:
            print('Calibrate back wall')
            return
        self.backward()
        self.forward(distance=constants.TO_CENTER)

    def wait_until_position(self, distance: int, stop_threshold: float = constants.PRE_TURN_THRESHOLD):
        """
        move mouse without resetting the distance already reached
        :param distance: distance to wait for
        :param stop_threshold: front sensor threshold to stop action
        """
        self._move(distance, stop_threshold)

    def stop(self):
        """
        call for stop motors until speed is zero
        """
        self._move_motors(0, 0)

    def right(self, angle: int = constants.TURN_90):
        """
        turn mouse in place right using gyro
        """
        self.reference_angle += angle
        self.update_sensor_data()
        debug(f'right {self.angle} -> {self.reference_angle}')
        while self.angle < self.reference_angle - constants.ANGLE_OFFSET:
            angle_diff = self.reference_angle - self.angle
            speed = constants.ROTATION_SPEED + angle_diff
            debug(speed)
            self._move_motors(speed, -speed)
            self.update_sensor_data()

    def left(self, angle: int = constants.TURN_90):
        """
        turn mouse in place left using gyro
        """
        self.reference_angle -= angle
        self.update_sensor_data()
        debug(f'left {self.angle} -> {self.reference_angle}')
        while self.angle > self.reference_angle + constants.ANGLE_OFFSET:
            angle_diff = self.angle - self.reference_angle
            speed = constants.ROTATION_SPEED + angle_diff
            debug(speed)
            self._move_motors(-speed, speed)
            self.update_sensor_data()

    def around(self):
        self.left()
        self.left()

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

    def _update_movement(self, left_enc: int, right_enc: int):
        """
        update movement related data i.e. position, angle, speed
        """
        diff = (left_enc * constants.LEFT_ENCODER_TO_MM + right_enc * constants.LEFT_ENCODER_TO_MM) / 2
        self.speed = diff / 0.1
        self.pos += diff

    def update_sensor_data(self, init_update: bool = False):
        """
        read sensor data and update wall related data
        """
        sensor_data = self.get_sensors()
        debug(str(sensor_data))
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
            left_enc_diff = 0
            right_enc_diff = 0
        else:
            lasers = sensor_data["laser"]
            self.front_wall_distance = lasers[Sensors.up]
            # TODO change sensors for P controller
            self.left_wall_distance = lasers[Sensors.left]
            self.right_wall_distance = lasers[Sensors.right]
            self.left_wall = self.left_wall_distance < constants.WALL_THRESHOLD
            self.front_wall = self.front_wall_distance < constants.FRONT_WALL_THRESHOLD
            self.right_wall = self.right_wall_distance < constants.WALL_THRESHOLD

            imu = sensor_data['imu']
            angle = imu["yaw"] - self.angle_error

            if init_update:
                self.angle_error = angle
                self.angle = 0

            if angle <= 90:
                if self.prev_angle >= 270:
                    self.prev_angle -= 360
            if angle >= 270:
                if self.prev_angle <= 90:
                    self.prev_angle += 360

            diff = angle - self.prev_angle
            self.prev_angle = angle
            self.angle += diff

            encoders = sensor_data['encoders']
            left_enc_diff = encoders['left_encoder_delta_sum']
            right_enc_diff = encoders['right_encoder_delta_sum']

        self._update_movement(left_enc=left_enc_diff, right_enc=right_enc_diff)
        self._calc_errors()
        debug(str(self))

    def __str__(self):
        return (
            f"< {self.left_wall_distance} {self.left_wall} \n"
            f"^ {self.front_wall_distance} {self.front_wall} \n"
            f"> {self.right_wall_distance} {self.right_wall} \n"
            f"pos: {self.pos} \n"
            f"angle: {self.angle} ({self.reference_angle}) \n"
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
