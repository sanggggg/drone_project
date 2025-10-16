#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import sys
import termios
import tty
import select
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from anafi_ros_interfaces.msg import MoveByCommand


HELP_TEXT = r"""
[ Anafi TestFlight – Keyboard Shortcuts ]

동작
  t : 이륙 ( /anafi/drone/takeoff )
  l : 착륙 ( /anafi/drone/land )
  k : HALT 강제 정지 ( /anafi/drone/halt )
  p : 즉시 호버링 (= 강제 정지와 동일 동작 호출)
  ? : 도움말 출력

시퀀스는 hovering/flying 상태에서 자동 시작되며,
키 입력은 언제든지 개입하여 즉시 수행됩니다.
Ctrl+C 로 종료
"""


def _make_qos(depth=10, reliable=True):
    return QoSProfile(
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
    )


class _Keyboard:
    """터미널 비차단 단일 문자 입력용 헬퍼"""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def getch(self) -> Optional[str]:
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            try:
                return sys.stdin.read(1)
            except Exception:
                return None
        return None

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class TestFlightNode(Node):
    def __init__(self):
        super().__init__('anafi_test_flight')

        # QoS
        qos_ctrl = _make_qos(depth=10, reliable=True)
        qos_sense = _make_qos(depth=10, reliable=False)

        # Pub/Sub
        self.pub_moveby = self.create_publisher(MoveByCommand, '/anafi/drone/moveby', qos_ctrl)
        self.sub_state = self.create_subscription(String, '/anafi/drone/state', self._on_state, qos_sense)
        self.sub_moveby_done = self.create_subscription(Bool, '/anafi/drone/moveby_done',
                                                        self._on_moveby_done, qos_ctrl)

        # Services
        self.cli_takeoff = self.create_client(Trigger, '/anafi/drone/takeoff')
        self.cli_land    = self.create_client(Trigger, '/anafi/drone/land')
        self.cli_halt    = self.create_client(Trigger, '/anafi/drone/halt')

        # 상태
        self._state = None
        self._started = False
        self._moveby_event = threading.Event()
        self._abort_flag = False

        # 키보드 입력 스레드
        self.get_logger().info(HELP_TEXT)
        self._kb = None
        try:
            self._kb = _Keyboard()
        except Exception as e:
            self.get_logger().warn(f"키보드 초기화 실패(tty 필요): {e!r}")

        if self._kb:
            self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._kb_thread.start()

        # 워커 스레드(시퀀스)
        self.get_logger().info('대기 중: 드론이 hovering/flying 이면 시퀀스를 시작합니다.')
        self._worker = threading.Thread(target=self._wait_and_run, daemon=True)
        self._worker.start()

    # ---------- Callbacks ----------
    def _on_state(self, msg: String):
        self._state = (msg.data or '').strip().lower()

    def _on_moveby_done(self, msg: Bool):
        # True/False 모두 이벤트는 set 하되, 실패 여부는 로그로 안내
        if msg.data:
            self.get_logger().info('MoveBy 완료 신호 수신 ✅')
        else:
            self.get_logger().warn('MoveBy 실패 신호 수신 ❌')
        self._moveby_event.set()

    # ---------- Service helpers ----------
    def _call_trigger(self, client, name: str, wait_timeout: float = 8.0) -> bool:
        if not client.service_is_ready():
            self.get_logger().info(f'waiting for {name} service ...')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'{name} 서비스가 준비되지 않았습니다.')
                return False
        try:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout)
            resp = future.result()
            if resp is None:
                self.get_logger().warn(f'{name} 응답 없음')
                return False
            if resp.success:
                self.get_logger().warning(f'{name}: {resp.message}')
                return True
            else:
                self.get_logger().warn(f'{name} 실패: {resp.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'{name} 호출 오류: {e!r}')
            return False

    def _call_takeoff(self):
        return self._call_trigger(self.cli_takeoff, 'takeoff')

    def _call_land(self):
        return self._call_trigger(self.cli_land, 'land')

    def _call_halt(self):
        # anafi.py의 halt: PCMD 0, RTH/POI/Mavlink stop 등 강제 정지
        return self._call_trigger(self.cli_halt, 'halt')

    # ---------- MoveBy ----------
    def _publish_moveby(self, dx: float, dy: float, dz: float = 0.0, dyaw: float = 0.0):
        msg = MoveByCommand()
        msg.dx = float(dx); msg.dy = float(dy); msg.dz = float(dz); msg.dyaw = float(dyaw)
        self.pub_moveby.publish(msg)
        self.get_logger().info(f'MoveBy PUB → dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.2f}')

    def _moveby_and_wait(self, dx, dy, dz=0.0, dyaw=0.0, timeout_sec=10.0) -> bool:
        if self._abort_flag:
            return False
        self._moveby_event.clear()
        self._publish_moveby(dx, dy, dz, dyaw)

        self.get_logger().info(f'MoveBy 완료 대기 (timeout {timeout_sec:.1f}s)...')
        ok = self._moveby_event.wait(timeout=timeout_sec)
        if not ok:
            self.get_logger().error('MoveBy 완료 신호 타임아웃 ⏰')
            return False
        return not self._abort_flag

    # ---------- Sleep ----------
    def _sleep(self, sec: float, note: str = ''):
        if note:
            self.get_logger().info(f'대기 {sec:.1f}s {note}')
        end = time.time() + sec
        while rclpy.ok() and time.time() < end and not self._abort_flag:
            time.sleep(0.05)

    # ---------- State helpers ----------
    def _is_hover_or_flying(self) -> bool:
        s = (self._state or '')
        return s in ('hovering', 'flying')

    # ---------- Keyboard ----------
    def _keyboard_loop(self):
        try:
            while rclpy.ok():
                ch = self._kb.getch()
                if ch is None:
                    time.sleep(0.01)
                    continue

                if ch == 't':
                    self.get_logger().warning('[키보드] 이륙 요청')
                    self._call_takeoff()

                elif ch == 'l':
                    self.get_logger().warning('[키보드] 착륙 요청')
                    self._call_land()

                elif ch == 'k':
                    self.get_logger().warning('[키보드] HALT 강제 정지 요청')
                    self._abort_flag = True   # 시퀀스 즉시 중단 유도
                    self._call_halt()

                elif ch == 'p':
                    self.get_logger().warning('[키보드] 즉시 호버(강제 정지) 요청')
                    self._abort_flag = True
                    self._call_halt()

                elif ch == '?':
                    self.get_logger().info(HELP_TEXT)
        finally:
            try:
                self._kb.restore()
            except Exception:
                pass

    # ---------- Main sequence ----------
    def _wait_and_run(self):
        # 시작 조건: hovering/flying
        self.get_logger().info('드론 상태 대기중... (hovering/flying)')
        while rclpy.ok() and not self._is_hover_or_flying() and not self._abort_flag:
            time.sleep(0.1)
        if not rclpy.ok() or self._started or self._abort_flag:
            return
        self._started = True

        self.get_logger().warning('시퀀스 시작!')
        self._sleep(0.5)

        # 1) 1.0 m 전진 (0.5 + 0.5)
        if not self._moveby_and_wait(0.5, 0.0): return self._abort_and_land()
        if not self._moveby_and_wait(1.0, 0.0): return self._abort_and_land()
        self._sleep(0.2)

        # 2) 호버 10 s
        # self._sleep(5.0, '(호버)')

        # 3) 0.5 m 전진
        if not self._moveby_and_wait(0.5, 0.0): return self._abort_and_land()

        # 4) 오른쪽 0.5 m
        if not self._moveby_and_wait(0.0, -0.5): return self._abort_and_land()

        # 5) 2.0 m 전진 (1.5)
        if not self._moveby_and_wait(1.0, 0.0): return self._abort_and_land()
        if not self._moveby_and_wait(0.7, 0.0): return self._abort_and_land()

        # 6) 왼쪽 0.5 m
        if not self._moveby_and_wait(0.0, 0.5): return self._abort_and_land()

        # 7) 1.5 m 전진 (0.3)  ※ 제공 스니펫에 맞춤
        if not self._moveby_and_wait(0.5, 0.0): return self._abort_and_land()

        # 8) 착륙
        self._call_land()
        self.get_logger().warning('시퀀스 종료')

    def _abort_and_land(self):
        self.get_logger().error('오류/중단으로 시퀀스 중단 → 착륙')
        # 중단 시에는 HALT로 즉시 정지 후 착륙을 권장
        self._call_halt()
        self._call_land()


def main():
    rclpy.init()
    node = TestFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 키보드 모드 원복
        if hasattr(node, "_kb") and node._kb:
            try:
                node._kb.restore()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
