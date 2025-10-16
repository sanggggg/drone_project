#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from anafi_ros_interfaces.msg import MoveByCommand


class TestFlightNode(Node):
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

        # ✅ 브릿지에서 퍼블리시하는 완료 신호 구독
        self.sub_moveby_done = self.create_subscription(Bool, '/anafi/drone/moveby_done',
                                                        self._on_moveby_done, qos_ctrl)

        self.cli_land = self.create_client(Trigger, '/anafi/drone/land')

        # 상태
        self._state = None
        self._started = False

        # ✅ 완료 대기용 이벤트
        self._moveby_event = threading.Event()

        self.get_logger().info('대기 중: 드론이 hovering/flying 이면 시퀀스를 시작합니다.')
        self._worker = threading.Thread(target=self._wait_and_run, daemon=True)
        self._worker.start()

    # ---------- Callbacks ----------
    def _on_state(self, msg: String):
        self._state = (msg.data or '').strip().lower()

    def _on_moveby_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('MoveBy 완료 신호 수신 ✅')
            self._moveby_event.set()
        else:
            self.get_logger().warn('MoveBy 실패 신호 수신 ❌')
            self._moveby_event.set()

    # ---------- Helpers ----------
    def _publish_moveby(self, dx: float, dy: float, dz: float = 0.0, dyaw: float = 0.0):
        msg = MoveByCommand()
        msg.dx = float(dx); msg.dy = float(dy); msg.dz = float(dz); msg.dyaw = float(dyaw)
        self.pub_moveby.publish(msg)
        self.get_logger().info(f'MoveBy PUB → dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.2f}')

    def _moveby_and_wait(self, dx, dy, dz=0.0, dyaw=0.0, timeout_sec=10.0) -> bool:
        """
        1) moveby publish
        2) /anafi/drone/moveby_done 이 True/False 올 때까지 대기 (timeout 포함)
        """
        self._moveby_event.clear()
        self._publish_moveby(dx, dy, dz, dyaw)

        self.get_logger().info(f'MoveBy 완료 대기 (timeout {timeout_sec:.1f}s)...')
        ok = self._moveby_event.wait(timeout=timeout_sec)
        if not ok:
            self.get_logger().error('MoveBy 완료 신호 타임아웃 ⏰')
            return False
        # 이벤트는 완료/실패 상관없이 set 되므로, 상태 판단은 로그/후속 검증으로 처리
        return True

    def _sleep(self, sec: float, note: str = ''):
        if note:
            self.get_logger().info(f'대기 {sec:.1f}s {note}')
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            time.sleep(0.05)

    def _call_land(self):
        if not self.cli_land.service_is_ready():
            self.get_logger().info('waiting for /drone/land service ...')
            self.cli_land.wait_for_service(timeout_sec=10.0)
        try:
            self.get_logger().warning('착륙 요청 /drone/land')
            future = self.cli_land.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'land service call failed: {e!r}')

    def _is_hover_or_flying(self) -> bool:
        s = (self._state or '')
        return s in ('hovering', 'flying')

    # ---------- Main sequence ----------
    def _wait_and_run(self):
        # 시작 조건: hovering/flying
        self.get_logger().info('드론 상태 대기중... (hovering/flying)')
        while rclpy.ok() and not self._is_hover_or_flying():
            time.sleep(0.1)
        if not rclpy.ok() or self._started:
            return
        self._started = True

        self.get_logger().warning('시퀀스 시작!')
        self._sleep(0.5)

        # 1) 1.0 m 전진
        if not self._moveby_and_wait(0.5, 0.0): return self._abort_and_land()
        self._sleep(0.2)

        # 2) 3.0 s 호버
        self._sleep(10.0, '(호버)')

        # 3) 0.5 m 전진
        if not self._moveby_and_wait(0.5, 0.0): return self._abort_and_land()

        # 4) 오른쪽 0.5 m
        if not self._moveby_and_wait(0.0, 0.5): return self._abort_and_land()

        # 5) 2.0 m 전진
        if not self._moveby_and_wait(1.5, 0.0): return self._abort_and_land()

        # 6) 왼쪽 0.5 m
        if not self._moveby_and_wait(0.0, -0.5): return self._abort_and_land()

        # 7) 1.5 m 전진
        if not self._moveby_and_wait(0.3, 0.0): return self._abort_and_land()

        # 8) 착륙
        self._call_land()
        self.get_logger().warning('시퀀스 종료')

    def _abort_and_land(self):
        self.get_logger().error('오류로 시퀀스 중단 → 착륙')
        self._call_land()


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
