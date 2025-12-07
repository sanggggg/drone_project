import time
import threading
import math

class WaypointExecutor:
    def __init__(
        self,
        *,
        get_odom,          # () -> (x, y, z, yaw) | None
        has_odom,          # () -> bool
        send_goto,         # (x, y, z, yaw) -> None
        logger,
        tolerance=0.15,
        poll_dt=0.1,
    ):
        self._get_odom = get_odom
        self._has_odom = has_odom
        self._send_goto = send_goto
        self._logger = logger

        self._tol = tolerance
        self._dt = poll_dt

        self._stop_event = threading.Event()
        self._running = False
        self._lock = threading.Lock()
        self.init_x = self._get_odom()[0]
        self.init_y = self._get_odom()[1]
        self.init_z = self._get_odom()[2]
        self.init_yaw = self._get_odom()[3]
        self._send_goto(self.init_x, self.init_y, self.init_z, self.init_yaw)
        self._waypoints = {
            '0': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x + 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5 , self.init_y, self.init_z - 1.0, self.init_yaw, 0),  # (1, -2)
                    (self.init_x, self.init_y, self.init_z - 1.0, self.init_yaw, 0),  # (0, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 1),  # 현재 위치 (x,z)
            ],
            '1': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x, self.init_y, self.init_z - 1.0, self.init_yaw, 1),  # (0, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '2': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x + 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5, self.init_y, self.init_z-0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x, self.init_y, self.init_z -0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x, self.init_y, self.init_z -1.0, self.init_yaw, 0),  # (0, -2)
                    (self.init_x + 0.5, self.init_y, self.init_z -1.0, self.init_yaw, 1),  # (1, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '3': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x + 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5, self.init_y, self.init_z-0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x, self.init_y, self.init_z -0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x + 0.5, self.init_y, self.init_z-0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x + 0.5, self.init_y, self.init_z-1.0, self.init_yaw, 0),  # (1, -2)
                    (self.init_x, self.init_y, self.init_z -1.0, self.init_yaw, 1),  # (0, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '4': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x + 0.5, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x + 0.5, self.init_y, self.init_z , self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5, self.init_y, self.init_z - 1.0 , self.init_yaw, 1),  # (1, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '5': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x - 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (-1, 0)
                    (self.init_x - 0.5, self.init_y, self.init_z-0.5, self.init_yaw, 0),  # (-1, -1)
                    (self.init_x, self.init_y, self.init_z -0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x, self.init_y, self.init_z -1.0, self.init_yaw, 0),  # (0, -2)
                    (self.init_x - 0.5, self.init_y, self.init_z -1.0, self.init_yaw, 1),  # (-1, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                ],
            '6': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x - 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (-1, 0)
                    (self.init_x - 0.5, self.init_y, self.init_z-1.0, self.init_yaw, 0),  # (-1, -2)
                    (self.init_x, self.init_y, self.init_z -1.0, self.init_yaw, 0),  # (0, -2)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (0, -1 )
                    (self.init_x - 0.5, self.init_y, self.init_z -0.5, self.init_yaw, 1),  # (-1, -1)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '7': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # 현재 위치 (0, -1)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x + 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5, self.init_y, self.init_z -1, self.init_yaw, 1),  # (1, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ],
            '8': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x + 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x + 0.5, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x, self.init_y, self.init_z - 1.0, self.init_yaw, 0),  # (0, -2)
                    (self.init_x + 0.5, self.init_y, self.init_z - 1.0, self.init_yaw, 0),  # (1, -2)
                    (self.init_x + 0.5, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 1),  # 현재 위치 (x,z)
            ],
            '9': [
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x - 0.5, self.init_y, self.init_z, self.init_yaw, 0),  # (1, 0)
                    (self.init_x - 0.5, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (1, -1)
                    (self.init_x, self.init_y, self.init_z - 0.5, self.init_yaw, 0),  # (0, -1)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
                    (self.init_x, self.init_y, self.init_z - 1.0, self.init_yaw, 1),  # (0, -2)
                    (self.init_x, self.init_y, self.init_z, self.init_yaw, 0),  # 현재 위치 (x,z)
            ]   
        }

    def wait_for_position(self, tx, ty, tz, timeout=10.0):
            start = time.monotonic()
            while time.monotonic() - start < timeout:
                if self._stop_event.is_set():
                    self._logger.info('WaypointExecutor: stop during wait')
                    return False

                if not self._has_odom():
                    time.sleep(self._dt)
                    continue

                x, y, z, _ = self._get_odom()
                if (
                    abs(x - tx) < self._tol and
                    abs(y - ty) < self._tol and
                    abs(z - tz) < self._tol
                ):
                    return True

                time.sleep(self._dt)

            self._logger.warn(f'WaypointExecutor: timeout at ({tx:.2f},{ty:.2f},{tz:.2f})')
            return False
    
    def run_sequence(self, digit_list):
        if self._running:
            self._logger.warn('WaypointExecutor already running')
            return

        threading.Thread(
            target=self._run_thread,
            args=(digit_list,),
            daemon=True
        ).start()

    def _run_thread(self, digit_list):
        with self._lock:
            self._running = True
            self._stop_event.clear()

        try:
            if not self._has_odom():
                self._logger.error('No odom, cannot start waypoint sequence')
                return

            self._logger.info('WaypointExecutor: starting in 3 seconds')
            for i in digit_list:
                waypoints = self._waypoints[i]

                for _ in range(10):
                    if self._stop_event.is_set():
                        return
                    time.sleep(0.1)

                for i, (x, y, z, yaw, wait_t) in enumerate(waypoints):
                    if self._stop_event.is_set():
                        return

                    self._logger.info(f'Waypoint {i+1}/{len(waypoints)}')

                    self._send_goto(x, y, z, yaw)

                    reached = self.wait_for_position(x, y, z)
                    if not reached:
                        self._logger.warn(f'Waypoint {i+1} not reached')

                    if wait_t > 0:
                        for _ in range(int(wait_t / self._dt)):
                            if self._stop_event.is_set():
                                return
                            time.sleep(self._dt)

            self._logger.info('WaypointExecutor: completed')

        finally:
            with self._lock:
                self._running = False


    def stop(self):
        self._stop_event.set()