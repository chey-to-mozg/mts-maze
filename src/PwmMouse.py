import time
from threading import Thread

import requests

from src import constants
from src.Mouse import Mouse, Sensors


class DataUpdater(Thread):
    def __init__(self, sim: bool = False):
        super().__init__()
        self._sim = sim
        self._api_route = "http://127.0.0.1:8801/api/v1" if sim else constants.MOUSE_IP
        self.angle_error = 0
        self.prev_angle = 0
        self.enc_left = 0
        self.enc_right = 0
        self.data = {'angle': 0}
        self.done = False
        self._init_angle()
        self._update_data()

    def _get_data(self):
        if self._sim:
            return requests.get(f'{self._api_route}/robot-cells/sensor-data?token={1}').json()
        else:
            body = {'id': constants.MOUSE_ID, 'type': 'all'}
            return requests.post(f"{self._api_route}/sensor", json=body).json()

    def _init_angle(self):
        sensor_data = self._get_data()
        if self._sim:
            self.angle_error = sensor_data['rotation_yaw']
        else:
            imu = sensor_data['imu']
            self.angle_error = imu["yaw"]

    def _update_data(self):
        sensor_data = self._get_data()
        if self._sim:
            left_wall_distance = sensor_data['left_side_distance']
            front_wall_distance = sensor_data['front_distance']
            right_wall_distance = sensor_data['right_side_distance']
            angle = sensor_data['rotation_yaw']
        else:
            lasers = sensor_data["laser"]
            front_wall_distance = lasers[Sensors.up]
            left_wall_distance = lasers[Sensors.left]
            right_wall_distance = lasers[Sensors.right]

            imu = sensor_data['imu']
            angle = imu['yaw'] - self.angle_error

        cur_angle = self.data['angle']

        if angle < -90:
            if self.prev_angle > 90:
                self.prev_angle -= 360
        if angle > 90:
            if self.prev_angle < -90:
                self.prev_angle += 360
        diff = angle - self.prev_angle
        cur_angle += diff
        self.prev_angle = angle

        # TODO add encoders update
        self.enc_left += 1
        self.enc_right += 1

        self.data = {
            'left': left_wall_distance,
            'front': front_wall_distance,
            'right': right_wall_distance,
            'angle': cur_angle,
            'enc_left': self.enc_left,
            'enc_right': self.enc_right,
        }

    def run(self) -> None:
        while not self.done:
            self._update_data()
            time.sleep(0.05)


class PwmMouse(Mouse):
    def __init__(self, sim: bool = False):
        super().__init__(sim)
        self.enc_left_prev = 0
        self.enc_right_prev = 0
        # add pwm variables for logging
        self.pwm_left = 0
        self.pwm_right = 0
        self.prev_update = time.time()
        self.updater = DataUpdater(sim)
        self.updater.start()

    def _move_motors(self, m_time: int = 100):
        """
        run motors with requested pwm and time
        :param m_time:    execution time
        """
        pwm_left = int(self.pwm_left)
        pwm_right = int(self.pwm_right)
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
            if self.front_wall_distance < stop_threshold:
                return
            if self.speed < self.max_speed:
                self.left_pwm = constants.FORWARD_SPEED + self.rot_error
                self.right_pwm = constants.FORWARD_SPEED - self.rot_error
                self._move_motors()
            else:
                # time.sleep(0.1)
                pass
            self.update_sensor_data()

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
        self.pwm_left = -constants.FORWARD_SPEED
        self.pwm_right = -constants.FORWARD_SPEED
        while self.speed > 5:
            self._move_motors()
            self.update_sensor_data()

    def forward(self, distance: int = constants.CELL):
        if self._sim:
            super().forward()
            return
        self.pos = 0
        self._move(distance, constants.FRONT_REFERENCE)

    def right(self, target_angle: int = constants.TURN_90):
        if self._sim:
            super().right()
            return
        self.reference_angle = self.reference_angle + target_angle
        self.pwm_left = constants.ROTATION_SPEED
        self.pwm_right = -constants.ROTATION_SPEED
        while self.angle < self.reference_angle - constants.ANGLE_OFFSET:
            self._move_motors()
            self.update_sensor_data()

    def left(self, target_angle: int = constants.TURN_90):
        if self._sim:
            super().left()
            return
        self.reference_angle = self.reference_angle - target_angle
        self.pwm_left = -constants.ROTATION_SPEED
        self.pwm_right = constants.ROTATION_SPEED
        while self.angle > self.reference_angle + constants.ANGLE_OFFSET:
            self._move_motors()
            self.update_sensor_data()

    def around_in_place(self):
        """
        turn around using 2 in place turns
        """
        self.left()
        self.left()

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

        pos_error = 0

        self.rot_error = ang_error - pos_error

    def _update_movement(self, enc_left: int, enc_right: int):
        diff_left = enc_left - self.enc_left_prev
        diff_right = enc_right - self.enc_right_prev
        distance = (diff_left * constants.MM_PER_COUNT_LEFT + diff_right * constants.MM_PER_COUNT_RIGHT) / 2

        self.pos += distance
        time_delta = time.time() - self.prev_update
        self.speed = distance / time_delta

        self.enc_left_prev = enc_left
        self.enc_right_prev = enc_right

    def update_sensor_data(self, init_update: bool = False):
        if init_update:
            return
        sensor_data = self.updater.data
        self.left_wall_distance = sensor_data['left']
        self.front_wall_distance = sensor_data['front']
        self.right_wall_distance = sensor_data['right']

        self.left_wall = self.left_wall_distance < constants.WALL_THRESHOLD
        self.front_wall = self.front_wall_distance < constants.FRONT_WALL_THRESHOLD
        self.right_wall = self.right_wall_distance < constants.WALL_THRESHOLD

        self.angle = sensor_data['angle']

        enc_left = sensor_data['enc_left']
        enc_right = sensor_data['enc_right']
        self._update_movement(enc_left, enc_right)
        self._calc_errors()

    def __str__(self):
        return (
            f"< {self.left_wall_distance} {self.left_wall} \n"
            f"^ {self.front_wall_distance} {self.front_wall} \n"
            f"> {self.right_wall_distance} {self.right_wall} \n"
            f"angle cur|ref: {self.angle} | {self.reference_angle} \n"
            f"position: {self.pos} \n"
            f"pwm left: {self.pwm_left} pwm_right: {self.pwm_right} speed: {round(self.speed)} mm / sec \n"
            f"rot error: {round(self.rot_error)}"
        )


if __name__ == '__main__':
    mouse = PwmMouse(sim=True)
    print(mouse)
    mouse.update_sensor_data()
    print(mouse)
    mouse.right()
    mouse.update_sensor_data()
    print(mouse)
    mouse.forward()
    mouse.update_sensor_data()
    print(mouse)
    mouse.updater.done = True
