#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from anafi_ros_interfaces.msg import MoveByCommand

HOVER_STATES = {'hovering'}  # 필요하면 'flying' 포함

class TestFlightNode(Node):
    """
    시나리오:
      (사용자가 수동으로 /drone/takeoff 호출하여 이륙 → 호버 상태 진입)
      1) 1.0 m 전진
      2) 3.0 s 호버(대기)
      3) 0.5 m 전진
      4) 0.3 m 오른쪽(우측)
      5) 0.5 m 전진
      6) 0.3 m 왼쪽(좌측)
      7) 1.0 m 전진
      8) 착륙(/drone/land 서비스 콜)
    """

    def __init__(self):
        super().__init__('anafi_test_flight')

        # QoS
        qos_ctrl = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        qos_sense = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
        )

        # Pub/Sub/Service
        self.pub_moveby = self.create_publisher(MoveByCommand, '/anafi/drone/moveby', qos_ctrl)
        self.sub_state = self.create_subscription(String, '/anafi/drone/state', self._on_state, qos_sense)
        self.cli_land = self.create_client(Trigger, '/anafi/drone/land')

        # 상태
        self._state = None
        self._started = False

        self.get_logger().info('TestFlightNode ready. 대기: 드론이 이륙해 hovering 상태가 되면 시퀀스를 시작합니다.')

        # 백그라운드 스레드로 시퀀스 시작 감시
        self._worker = threading.Thread(target=self._wait_and_run, daemon=True)
        self._worker.start()

    # ---------- Callbacks ----------
    def _on_state(self, msg: String):
        self._state = (msg.data or '').strip().lower()

    # ---------- Helpers ----------
    def _publish_moveby(self, dx: float, dy: float, dz: float = 0.0, dyaw: float = 0.0):
        """
        MoveByCommand:
          dx: 앞(+)/뒤(-) [m]
          dy: 오른쪽(+)/왼쪽(-) [m]
          dz: 아래(+)/위(-) [m]  (우리는 0.0)
          dyaw: 라디안(+는 좌회전) (우리는 0.0)
        """
        msg = MoveByCommand()
        msg.dx = float(dx)
        msg.dy = float(dy)
        msg.dz = float(dz)
        msg.dyaw = float(dyaw)
        self.pub_moveby.publish(msg)
        self.get_logger().info(f'MoveBy: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.2f}')

    def _sleep(self, sec: float, note: str = ''):
        if note:
            self.get_logger().info(f'대기 {sec:.1f}s {note}')
        end = time.time() + sec
        # rclpy 종료 대응
        while rclpy.ok() and time.time() < end:
            time.sleep(0.05)

    def _call_land(self):
        if not self.cli_land.service_is_ready():
            self.get_logger().info('waiting for /drone/land service ...')
            self.cli_land.wait_for_service(timeout_sec=10.0)
        try:
            self.get_logger().warning('착륙 요청 /drone/land')
            future = self.cli_land.call_async(Trigger.Request())
            # 굳이 결과를 기다리지 않아도 되지만, 5초만 기다려봄
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'land service call failed: {e!r}')

    def _is_hover_or_flying(self) -> bool:
        s = (self._state or '')
        return s in ('hovering', 'flying')

    

    def _wait_hover(self, timeout: float = 10.0) -> bool:
        """브릿지의 extended_move_by가 끝나면 FS가 hovering으로 돌아옵니다."""
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout:
            if (self._state or '').lower() in HOVER_STATES:
                return True
            time.sleep(0.05)
        self.get_logger().warn('hovering 대기 타임아웃(계속 진행)')
        return False

    def _publish_moveby_and_wait(self, dx, dy, dz=0.0, dyaw=0.0, note=''):
        self._publish_moveby(dx, dy, dz, dyaw)
        if note:
            self.get_logger().info(note)
        # 브릿지가 .wait()로 끝낼 때까지 상태가 hovering으로 복귀하는 걸 기다림
        self._wait_hover(timeout=20.0)

    # ---------- Main sequence ----------
    def _wait_and_run(self):
        # 1) 시작 조건: hovering/flying 상태가 될 때까지 대기
        self.get_logger().info('드론 상태 대기중... (hovering/flying)')
        while rclpy.ok() and not self._is_hover_or_flying():
            time.sleep(0.1)

        if not rclpy.ok():
            return

        if self._started:
            return
        self._started = True

        self.get_logger().warning('시퀀스 시작!')

        # 안전을 위해 아주 짧게 대기(이륙 직후 안정화)
        self._sleep(0.5)

        # # --- 시퀀스 ---
        # # 1) 1.0 m 전진
        # self._publish_moveby(dx=1.0, dy=0.0)
        # self._sleep(2.0, '(이동 완료 대기)')  # 기본 수평속도 1m/s 가정, 여유 포함

        # # 2) 3.0 s 호버
        # self._sleep(8.0, '(호버)')

        # # 3) 0.5 m 전진
        # self._publish_moveby(dx=0.5, dy=0.0)
        # self._sleep(1.2, '(이동 완료 대기)')

        # # 4) 오른쪽 0.3 m
        # self._publish_moveby(dx=0.0, dy=0.8)
        # self._sleep(1.5, '(이동 완료 대기)')

        # # 5) 0.5 m 전진
        # self._publish_moveby(dx=1.0, dy=0.0)
        # self._sleep(1.2, '(이동 완료 대기)')

        # # 6) 왼쪽 0.3 m
        # self._publish_moveby(dx=0.0, dy=-0.8)
        # self._sleep(1.5, '(이동 완료 대기)')

        # # 7) 1.0 m 전진
        # self._publish_moveby(dx=1.0, dy=0.0)
        # self._sleep(3.0, '(이동 완료 대기)')

        # 1) 0.5 m 전진
        self._publish_moveby_and_wait(0.5, 0.0, note='(1/7)')
        # 2) 8s 호버(그냥 대기)
        self._sleep(8.0, '(2/7 호버)')
        # 3) 0.5 m 전진
        self._publish_moveby_and_wait(0.5, 0.0, note='(3/7)')
        # 4) 오른쪽 0.4 m
        self._publish_moveby_and_wait(0.0, 0.4, note='(4/7)')
        # 5) 0.5 m 전진
        self._publish_moveby_and_wait(0.5, 0.0, note='(5/7)')
        # 6) 왼쪽 0.4 m
        self._publish_moveby_and_wait(0.0, -0.4, note='(6/7)')
        # 7) 0.7 m 전진
        self._publish_moveby_and_wait(0.7, 0.0, note='(7/7)')



        # 8) 착륙
        self._call_land()

        self.get_logger().warning('시퀀스 종료')


def main():
    rclpy.init()
    node = TestFlightNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
