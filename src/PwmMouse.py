import time
from multiprocessing import Process, Value

import requests

from src import constants
from src.Mouse import Mouse, Sensors


class DataUpdater(Process):
    def __init__(self, update_delay: float, sim: bool = False):
        super().__init__()
        self._sim = sim
        self.update_delay = update_delay
        self._api_route = "http://127.0.0.1:8801/api/v1" if sim else constants.MOUSE_IP
        self.angle_error = 0
        self.prev_angle = 0
        self._done = Value('i', 0)
        self._angle = Value('i', 0)
        self._left_wall = Value('i', 0)
        self._front_wall = Value('i', 0)
        self._right_wall = Value('i', 0)
        self._left_encoder = Value('i', 0)
        self._right_encoder = Value('i', 0)
        self._init_angle()
        self._update_data()

    def get_angle(self):
        return self._angle.value

    def get_left_wall(self):
        return self._left_wall.value

    def get_front_wall(self):
        return self._front_wall.value

    def get_right_wall(self):
        return self._right_wall.value

    def get_left_encoder(self):
        return self._left_encoder.value

    def get_right_encoder(self):
        return self._right_encoder.value

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

        cur_angle = self.get_angle()

        if self._sim:
            if angle <= -90:
                if self.prev_angle >= 90:
                    self.prev_angle -= 360
            if angle >= 90:
                if self.prev_angle <= -90:
                    self.prev_angle += 360
        else:
            if angle <= 90:
                if self.prev_angle >= 270:
                    self.prev_angle -= 360
            if angle >= 270:
                if self.prev_angle <= 90:
                    self.prev_angle += 360
        diff = angle - self.prev_angle
        cur_angle += diff
        self.prev_angle = angle

        # TODO add encoders update
        enc_left = 1
        enc_right = 1

        self._angle.value = cur_angle
        self._left_wall.value = int(left_wall_distance)
        self._front_wall.value = int(front_wall_distance)
        self._right_wall.value = int(right_wall_distance)
        self._left_encoder.value = enc_left
        self._right_encoder.value = enc_right

    def break_updater(self):
        self._done.value = 1

    def run(self) -> None:
        while not self._done.value:
            self._update_data()
            time.sleep(self.update_delay)


class PwmController(Process):
    def __init__(self):
        super().__init__()
        self.pwm_left = Value('i', 0)
        self.pwm_right = Value('i', 0)
        self._done = Value('i', 0)
        self._run_pwm = Value('i', 0)

    def start_pwm(self, pwm_left: float | int, pwm_right: float | int):
        self._run_pwm.value = 0
        self.pwm_left.value = int(pwm_left)
        self.pwm_right.value = int(pwm_right)
        self._run_pwm.value = 1

    def stop_pwm(self):
        self._run_pwm.value = 0

        pwm_left = -5
        pwm_right = -5
        # need to send pulse in another direction
        if self.pwm_right.value < 0:
            pwm_right *= -1
        if self.pwm_left.value < 0:
            pwm_left *= -1
        self.pwm_left.value = pwm_left
        self.pwm_right.value = pwm_right

        self._run_pwm.value = 1
        time.sleep(0.1)
        self._run_pwm.value = 0

    def _move_motors(self, m_time: int = 100):
        """
        run motors with requested pwm and time
        :param m_time:    execution time
        """
        start = time.time()
        body = {
            'id': constants.MOUSE_ID,
            'l': self.pwm_left.value,
            'r': self.pwm_right.value,
            'l_time': m_time,
            'r_time': m_time,
        }
        requests.put(f"{constants.MOUSE_IP}/motor", json=body)
        diff = time.time() - start  # seconds
        sleep_time = m_time / 1000 - diff
        time.sleep(sleep_time)

    def break_pwm(self):
        self._done.value = 1

    def run(self):
        while not self._done.value:
            if self._run_pwm.value:
                self._move_motors()
            else:
                print('no pwm')
                time.sleep(0.01)


class PwmMouse(Mouse):
    def __init__(self, update_delay: float, sim: bool = False, calibrate_on_start: bool = False):
        super().__init__(sim)
        self.enc_left_prev = 0
        self.enc_right_prev = 0
        # add pwm variables for logging
        self.pwm_left = 0
        self.pwm_right = 0
        if calibrate_on_start:
            self.calibrate_back_wall()
        self.updater = DataUpdater(update_delay=update_delay, sim=sim)
        self.updater.start()
        self.pwm = PwmController()
        self.pwm.start()
        self.prev_update = time.time()

    def _start_pwm(self, pwm_left: float | int, pwm_right: float | int):
        self.pwm_left = pwm_left
        self.pwm_right = pwm_right
        self.pwm.start_pwm(pwm_left, pwm_right)

    def _move(self, dist: float, stop_threshold: float):
        """
        go forward until dist reached
        :param dist: distance to go
        :param stop_threshold: front sensor threshold to stop action
        """
        self.update_sensor_data()
        left_pwm = constants.FORWARD_SPEED + self.rot_error
        right_pwm = constants.FORWARD_SPEED - self.rot_error
        self._start_pwm(left_pwm, right_pwm)
        while self.pos < dist:
            self.update_sensor_data()
            if self.front_wall_distance < stop_threshold:
                break
        self.stop()

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
        self.pwm.stop_pwm()

    def soft_stop(self):
        self.pwm.start_pwm(0, 0)

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
        target = self.reference_angle - constants.ANGLE_OFFSET
        self._start_pwm(pwm_left=constants.ROTATION_SPEED, pwm_right=-constants.ROTATION_SPEED)

        while angle := self.updater.get_angle() < target:
            print(f'{angle} | {target}')
            # self.update_sensor_data()
        self.stop()

    def left(self, target_angle: int = constants.TURN_90):
        if self._sim:
            super().left()
            return

        self.reference_angle = self.reference_angle - target_angle
        target = self.reference_angle + constants.ANGLE_OFFSET
        self._start_pwm(pwm_left=-constants.ROTATION_SPEED, pwm_right=constants.ROTATION_SPEED)

        while angle := self.updater.get_angle() > target:
            print(f'{angle} | {target}')
            # self.update_sensor_data()
        self.stop()

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
        ang_error = (self.reference_angle - self.updater.get_angle()) * constants.KP_ROT

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

    def _update_movement(self):
        enc_left = self.updater.get_left_encoder()
        enc_right = self.updater.get_right_encoder()
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
        self.left_wall_distance = self.updater.get_left_wall()
        self.front_wall_distance = self.updater.get_front_wall()
        self.right_wall_distance = self.updater.get_right_wall()

        self.left_wall = self.left_wall_distance < constants.WALL_THRESHOLD
        self.front_wall = self.front_wall_distance < constants.FRONT_WALL_THRESHOLD
        self.right_wall = self.right_wall_distance < constants.WALL_THRESHOLD

        self._update_movement()
        self._calc_errors()
        print(self)

    def break_mouse(self):
        self.pwm.break_pwm()
        self.updater.break_updater()

    def __str__(self):
        return (
            f"< {self.left_wall_distance} {self.left_wall} \n"
            f"^ {self.front_wall_distance} {self.front_wall} \n"
            f"> {self.right_wall_distance} {self.right_wall} \n"
            f"angle cur|ref: {self.updater.get_angle()} | {self.reference_angle} \n"
            f"position: {self.pos} \n"
            f"pwm left: {self.pwm_left} pwm_right: {self.pwm_right} speed: {round(self.speed)} mm / sec \n"
            f"rot error: {round(self.rot_error)}"
        )


def check_turns():
    mouse = PwmMouse(update_delay=0.04, sim=True)
    mouse.right()
    # check inertia
    time.sleep(0.5)
    print(mouse)

    mouse.right()
    time.sleep(0.5)
    print(mouse)

    mouse.left(90)
    mouse.left(90)
    time.sleep(0.5)
    print(mouse)

    mouse.left()
    time.sleep(0.5)
    print(mouse)

    mouse.left()
    time.sleep(0.5)
    print(mouse)

    mouse.right(90)
    mouse.right(90)
    time.sleep(0.5)
    print(mouse)

    mouse.break_mouse()


def check_forward():
    mouse = PwmMouse(update_delay=0.04)

    mouse.forward()
    # check inertia
    print(mouse.get_sensors())
    time.sleep(0.5)
    print(mouse.get_sensors())

    mouse.forward(constants.CELL * 2)
    print(mouse.get_sensors())
    time.sleep(0.5)
    print(mouse.get_sensors())

    mouse.break_mouse()


def check_pwm():
    mouse = PwmMouse(update_delay=1, sim=True)
    mouse.pwm.start_pwm(100, 100)
    time.sleep(1)
    mouse.stop()
    mouse.break_mouse()


if __name__ == '__main__':
    # check_turns()
    # check_forward()
    check_pwm()
