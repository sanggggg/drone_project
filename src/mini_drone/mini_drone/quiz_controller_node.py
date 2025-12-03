#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quiz Controller Node

상태 머신 기반의 퀴즈 데모 컨트롤러.
ANAFI와 Crazyflie 두 드론을 조율하여 퀴즈 탐지 및 궤적 그리기를 수행합니다.

States:
    UNINIT   - 모든 드론이 땅에 있는 초기 상태
    IDLE     - 모든 드론이 takeoff하여 홈 위치에 있는 상태
    DETECTING - ANAFI가 퀴즈를 탐지하는 상태
    DRAWING  - Mini drone이 답에 해당하는 궤적을 그리는 상태
    FINISH   - 모든 드론이 착륙한 최종 상태

Keyboard Controls:
    's' - UnInit→Idle (시작) / Drawing→Idle (복귀)
    'n' - Idle→Detecting (탐지 시작)
    'x' - 종료 (→Finish)
    '1'/'2' - (mini_only_mode + DETECTING) 수동 answer 입력
    SPACE - Emergency Stop (모든 드론 즉시 정지)

Parameters:
    mini_only_mode: ANAFI 없이 Mini drone만 테스트하는 모드
"""

import sys
import termios
import tty
import select
import math
import threading
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from mini_drone_interfaces.srv import RunTrajectory


class QuizState(Enum):
    """퀴즈 컨트롤러 상태"""
    UNINIT = auto()
    IDLE = auto()
    DETECTING = auto()
    DRAWING = auto()
    FINISH = auto()


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion -> yaw (rad)"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> tuple:
    """yaw (rad) -> Quaternion (x, y, z, w)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


class QuizControllerNode(Node):
    """
    Quiz Demo 전체를 조율하는 상태 머신 기반 컨트롤러

    키보드 입력:
      's' - UnInit→Idle (시작) / Drawing→Idle (복귀)
      'n' - Idle→Detecting (탐지 시작)
      'x' - 종료 (→Finish)
    """

    def __init__(self):
        super().__init__('quiz_controller')

        # ---- Parameters ----
        # Mini drone 홈 위치
        self.declare_parameter('mini_home_x', 0.0)
        self.declare_parameter('mini_home_y', 0.0)
        self.declare_parameter('mini_home_z', 0.4)
        self.declare_parameter('mini_home_yaw_deg', 0.0)

        # ANAFI 홈 위치 (로컬 좌표계)
        self.declare_parameter('anafi_home_x', 0.0)
        self.declare_parameter('anafi_home_y', 0.0)
        self.declare_parameter('anafi_home_z', 0.6)
        self.declare_parameter('anafi_home_yaw_deg', 0.0)

        # 타이밍
        self.declare_parameter('takeoff_duration_s', 2.0)
        self.declare_parameter('land_duration_s', 2.0)
        self.declare_parameter('command_settle_time_s', 0.2)
        self.declare_parameter('operation_timeout_min', 5.0)

        # 홈 도달 판정
        self.declare_parameter('home_position_tolerance_m', 0.1)
        self.declare_parameter('home_yaw_tolerance_deg', 15.0)

        # Mini only mode (ANAFI 없이 테스트)
        self.declare_parameter('mini_only_mode', False)

        # 파라미터 로드
        self.mini_home_x = float(self.get_parameter('mini_home_x').value)
        self.mini_home_y = float(self.get_parameter('mini_home_y').value)
        self.mini_home_z = float(self.get_parameter('mini_home_z').value)
        self.mini_home_yaw = math.radians(float(self.get_parameter('mini_home_yaw_deg').value))

        self.anafi_home_x = float(self.get_parameter('anafi_home_x').value)
        self.anafi_home_y = float(self.get_parameter('anafi_home_y').value)
        self.anafi_home_z = float(self.get_parameter('anafi_home_z').value)
        self.anafi_home_yaw = math.radians(float(self.get_parameter('anafi_home_yaw_deg').value))

        self.takeoff_duration = float(self.get_parameter('takeoff_duration_s').value)
        self.land_duration = float(self.get_parameter('land_duration_s').value)
        self.settle_time = float(self.get_parameter('command_settle_time_s').value)
        self.operation_timeout = float(self.get_parameter('operation_timeout_min').value) * 60.0

        self.home_pos_tol = float(self.get_parameter('home_position_tolerance_m').value)
        self.home_yaw_tol = math.radians(float(self.get_parameter('home_yaw_tolerance_deg').value))

        self.mini_only_mode = bool(self.get_parameter('mini_only_mode').value)

        # Answer → Trajectory 매핑
        self.answer_trajectory_map = {
            "1": "figure8",
            "2": "vertical_a",
        }

        # ---- QoS Profiles ----
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable.history = HistoryPolicy.KEEP_LAST

        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best_effort.history = HistoryPolicy.KEEP_LAST

        # ---- State ----
        self._lock = threading.Lock()
        self.state = QuizState.UNINIT
        self.operation_start_time: Optional[float] = None
        self.pending_home_return = False
        self.pending_transition: Optional[tuple] = None  # (target_state, transition_time)

        # Mini drone odom
        self._mini_x = 0.0
        self._mini_y = 0.0
        self._mini_z = 0.0
        self._mini_yaw = 0.0
        self._have_mini_odom = False

        # ANAFI state
        self._anafi_state = "unknown"

        # ---- Subscribers ----
        self.sub_cf_odom = self.create_subscription(
            Odometry, '/cf/odom', self._cf_odom_cb, qos_best_effort
        )
        self.sub_anafi_state = self.create_subscription(
            String, '/anafi/drone/state', self._anafi_state_cb, qos_best_effort
        )
        self.sub_quiz_answer = self.create_subscription(
            String, '/quiz/answer', self._quiz_answer_cb, qos_reliable
        )

        # ---- Publishers ----
        self.pub_state = self.create_publisher(String, '/quiz/state', qos_reliable)
        self.pub_cf_takeoff = self.create_publisher(Float32, '/cf/hl/takeoff', qos_reliable)
        self.pub_cf_land = self.create_publisher(Float32, '/cf/hl/land', qos_reliable)
        self.pub_cf_goto = self.create_publisher(PoseStamped, '/cf/hl/goto', qos_reliable)
        self.pub_quiz_answer = self.create_publisher(String, '/quiz/answer', qos_reliable)

        # ---- Service Clients ----
        self.cli_anafi_takeoff = self.create_client(Trigger, '/anafi/drone/takeoff')
        self.cli_anafi_land = self.create_client(Trigger, '/anafi/drone/land')
        self.cli_anafi_emergency = self.create_client(Trigger, '/anafi/drone/emergency')
        self.cli_cf_stop = self.create_client(Trigger, '/cf/stop')
        self.cli_cf_traj = self.create_client(RunTrajectory, '/cf/traj/run')

        # ---- Timers ----
        self.create_timer(0.1, self._check_state_timer)  # 10Hz 상태 체크
        self.create_timer(1.0, self._publish_state_timer)  # 1Hz 상태 발행

        # ---- Keyboard Thread ----
        self._running = True
        self._key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._key_thread.start()

        self._print_ui()
        mode_str = "[MINI-ONLY MODE]" if self.mini_only_mode else "[DUAL-DRONE MODE]"
        self.get_logger().info(f'Quiz Controller initialized {mode_str}. State: UNINIT')

    # ==================== Callbacks ====================

    def _cf_odom_cb(self, msg: Odometry):
        """Crazyflie odometry 콜백"""
        with self._lock:
            self._have_mini_odom = True
            self._mini_x = msg.pose.pose.position.x
            self._mini_y = msg.pose.pose.position.y
            self._mini_z = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._mini_yaw = quat_to_yaw(qx, qy, qz, qw)

    def _anafi_state_cb(self, msg: String):
        """ANAFI 상태 콜백"""
        with self._lock:
            self._anafi_state = msg.data

    def _quiz_answer_cb(self, msg: String):
        """퀴즈 답변 수신 콜백 - Detecting→Drawing 전환 트리거"""
        with self._lock:
            if self.state != QuizState.DETECTING:
                self.get_logger().debug(f"Ignored quiz answer in state {self.state.name}")
                return

            answer = msg.data.strip()
            trajectory = self.answer_trajectory_map.get(answer)

            if trajectory is None:
                self.get_logger().warn(f"Unknown answer: '{answer}'. Available: {list(self.answer_trajectory_map.keys())}")
                return

            self.get_logger().info(f"Quiz answer received: '{answer}' → trajectory: '{trajectory}'")
            self._run_trajectory(trajectory)
            self.state = QuizState.DRAWING
            self._update_ui()

    # ==================== Timer Callbacks ====================

    def _check_state_timer(self):
        """10Hz 상태 체크 타이머"""
        with self._lock:
            # 5분 타임아웃 체크
            if self.operation_start_time is not None:
                elapsed = time.time() - self.operation_start_time
                if elapsed >= self.operation_timeout:
                    if self.state in (QuizState.IDLE, QuizState.DETECTING, QuizState.DRAWING):
                        self.get_logger().warn(f"Operation timeout ({self.operation_timeout/60:.0f} min) reached!")
                        self._initiate_finish_locked()
                        return

            # 예약된 전환 체크
            if self.pending_transition is not None:
                target_state, transition_time = self.pending_transition
                if time.time() >= transition_time:
                    self.state = target_state
                    self.pending_transition = None
                    self.get_logger().info(f"State transition complete → {target_state.name}")
                    self._update_ui()
                    return

            # Drawing→Idle 전환 체크 (홈 도달 확인)
            if self.pending_home_return and self._is_mini_at_home_locked():
                self.pending_home_return = False
                self.state = QuizState.IDLE
                self.get_logger().info("Mini drone returned home → IDLE")
                self._update_ui()

    def _publish_state_timer(self):
        """1Hz 상태 발행 타이머"""
        with self._lock:
            state_name = self.state.name
        msg = String()
        msg.data = state_name
        self.pub_state.publish(msg)

    # ==================== State Helpers ====================

    def _is_mini_at_home_locked(self) -> bool:
        """Mini drone이 홈 위치에 도달했는지 확인 (lock 보유 상태에서 호출)"""
        if not self._have_mini_odom:
            return False

        dx = self._mini_x - self.mini_home_x
        dy = self._mini_y - self.mini_home_y
        dz = self._mini_z - self.mini_home_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # yaw 차이 (wrap-around 고려)
        dyaw = abs(self._mini_yaw - self.mini_home_yaw)
        dyaw = min(dyaw, 2*math.pi - dyaw)

        return dist <= self.home_pos_tol and dyaw <= self.home_yaw_tol

    def _schedule_transition(self, target_state: QuizState, delay: float):
        """지연된 상태 전환 예약 (lock 보유 상태에서 호출)"""
        self.pending_transition = (target_state, time.time() + delay)
        self.get_logger().info(f"Scheduled transition to {target_state.name} in {delay:.1f}s")

    # ==================== Drone Commands ====================

    def _takeoff_both(self):
        """두 드론 동시 takeoff (mini_only_mode면 Mini만)"""
        # ANAFI takeoff (mini_only_mode가 아닐 때만)
        if not self.mini_only_mode:
            if self.cli_anafi_takeoff.service_is_ready():
                req = Trigger.Request()
                future = self.cli_anafi_takeoff.call_async(req)
                future.add_done_callback(
                    lambda f: self.get_logger().info(f"ANAFI takeoff: {f.result().message if f.result() else 'failed'}")
                )
            else:
                self.get_logger().warn("ANAFI takeoff service not ready")
        else:
            self.get_logger().info("[MINI-ONLY] Skipping ANAFI takeoff")

        # Mini takeoff
        msg = Float32()
        msg.data = self.mini_home_z
        self.pub_cf_takeoff.publish(msg)
        self.get_logger().info(f"Mini takeoff command sent (target z={self.mini_home_z}m)")

    def _land_both(self):
        """두 드론 동시 착륙 (mini_only_mode면 Mini만)"""
        # ANAFI land (mini_only_mode가 아닐 때만)
        if not self.mini_only_mode:
            if self.cli_anafi_land.service_is_ready():
                req = Trigger.Request()
                future = self.cli_anafi_land.call_async(req)
                future.add_done_callback(
                    lambda f: self.get_logger().info(f"ANAFI land: {f.result().message if f.result() else 'failed'}")
                )
            else:
                self.get_logger().warn("ANAFI land service not ready")
        else:
            self.get_logger().info("[MINI-ONLY] Skipping ANAFI land")

        # Mini land
        msg = Float32()
        msg.data = 0.0
        self.pub_cf_land.publish(msg)
        self.get_logger().info("Mini land command sent")

    def _handle_emergency_stop(self):
        """Emergency Stop - 모든 드론 즉시 정지"""
        self.get_logger().warn("!!! EMERGENCY STOP !!!")

        # Mini drone E-STOP
        if self.cli_cf_stop.service_is_ready():
            req = Trigger.Request()
            future = self.cli_cf_stop.call_async(req)
            future.add_done_callback(
                lambda f: self.get_logger().warn(f"Mini E-STOP: {f.result().message if f.result() else 'failed'}")
            )
        else:
            self.get_logger().warn("Mini E-STOP service not ready")

        # ANAFI emergency (mini_only_mode가 아닐 때만)
        if not self.mini_only_mode:
            if self.cli_anafi_emergency.service_is_ready():
                req = Trigger.Request()
                future = self.cli_anafi_emergency.call_async(req)
                future.add_done_callback(
                    lambda f: self.get_logger().warn(f"ANAFI E-STOP: {f.result().message if f.result() else 'failed'}")
                )
            else:
                self.get_logger().warn("ANAFI emergency service not ready")

        # 상태를 FINISH로 전환
        with self._lock:
            self.state = QuizState.FINISH
            self.pending_transition = None
            self.pending_home_return = False
            self._update_ui()

    def _goto_mini_home(self):
        """Mini drone을 홈 위치로 이동"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.mini_home_x
        msg.pose.position.y = self.mini_home_y
        msg.pose.position.z = self.mini_home_z

        qx, qy, qz, qw = yaw_to_quat(self.mini_home_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pub_cf_goto.publish(msg)
        self.get_logger().info(
            f"Mini goto home: ({self.mini_home_x}, {self.mini_home_y}, {self.mini_home_z}), "
            f"yaw={math.degrees(self.mini_home_yaw):.1f}deg"
        )

    def _run_trajectory(self, trajectory_type: str):
        """Mini drone 궤적 실행"""
        if not self.cli_cf_traj.service_is_ready():
            self.get_logger().warn("Trajectory service not ready")
            return

        req = RunTrajectory.Request()
        req.trajectory_type = trajectory_type
        future = self.cli_cf_traj.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"Trajectory '{trajectory_type}': {resp.message}")
            except Exception as e:
                self.get_logger().error(f"Trajectory call failed: {e}")

        future.add_done_callback(_done_cb)

    # ==================== Key Handling ====================

    def _on_key_press(self, key: str):
        """키 입력 처리"""
        with self._lock:
            current_state = self.state

            if key == 's':
                if current_state == QuizState.UNINIT:
                    self._handle_start_locked()
                elif current_state == QuizState.DRAWING:
                    self._handle_return_home_locked()
                else:
                    self.get_logger().debug(f"Key 's' ignored in state {current_state.name}")

            elif key == 'n':
                if current_state == QuizState.IDLE:
                    self._handle_start_detecting_locked()
                else:
                    self.get_logger().debug(f"Key 'n' ignored in state {current_state.name}")

            elif key == 'x':
                if current_state in (QuizState.IDLE, QuizState.DETECTING, QuizState.DRAWING):
                    self._initiate_finish_locked()
                else:
                    self.get_logger().debug(f"Key 'x' ignored in state {current_state.name}")

            # mini_only_mode에서 Detecting 상태일 때 '1', '2' 키로 수동 answer 입력
            elif key in ('1', '2'):
                if self.mini_only_mode and current_state == QuizState.DETECTING:
                    self._handle_manual_answer_locked(key)
                else:
                    self.get_logger().debug(f"Key '{key}' ignored (mini_only_mode={self.mini_only_mode}, state={current_state.name})")

            # Space: Emergency Stop
            elif key == ' ':
                self._handle_emergency_stop()

    def _handle_start_locked(self):
        """UnInit → Idle 전환 처리 (lock 보유 상태)"""
        self.get_logger().info("Starting operation: Takeoff both drones...")
        self._takeoff_both()
        self.operation_start_time = time.time()
        delay = self.takeoff_duration + self.settle_time
        self._schedule_transition(QuizState.IDLE, delay)
        self._update_ui()

    def _handle_start_detecting_locked(self):
        """Idle → Detecting 전환 처리 (lock 보유 상태)"""
        self.state = QuizState.DETECTING
        if self.mini_only_mode:
            self.get_logger().info("Detecting started - press [1] or [2] for manual answer")
        else:
            self.get_logger().info("Detecting started - waiting for /quiz/answer...")
        self._update_ui()

    def _handle_manual_answer_locked(self, answer: str):
        """mini_only_mode에서 수동 answer 입력 처리 (lock 보유 상태)"""
        trajectory = self.answer_trajectory_map.get(answer)
        if trajectory is None:
            self.get_logger().warn(f"Unknown answer: '{answer}'")
            return

        self.get_logger().info(f"[MINI-ONLY] Manual answer: '{answer}' → trajectory: '{trajectory}'")

        # /quiz/answer 토픽 발행 (다른 노드가 구독 중일 수 있음)
        msg = String()
        msg.data = answer
        self.pub_quiz_answer.publish(msg)

        # 직접 Drawing 상태로 전환 및 궤적 실행
        self._run_trajectory(trajectory)
        self.state = QuizState.DRAWING
        self._update_ui()

    def _handle_return_home_locked(self):
        """Drawing 중 's' 키: 홈으로 복귀 (lock 보유 상태)"""
        self.get_logger().info("Returning mini drone to home...")
        self._goto_mini_home()
        self.pending_home_return = True
        self._update_ui()

    def _initiate_finish_locked(self):
        """착륙 시작 및 Finish 전환 예약 (lock 보유 상태)"""
        self.get_logger().info("Finishing operation: Landing both drones...")
        self._land_both()
        delay = self.land_duration + self.settle_time
        self._schedule_transition(QuizState.FINISH, delay)
        self.pending_home_return = False  # 홈 복귀 대기 취소
        self._update_ui()

    # ==================== Keyboard Loop ====================

    def _get_key(self, timeout: float = 0.1) -> Optional[str]:
        """Non-blocking key read (Linux)"""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _keyboard_loop(self):
        """별도 스레드에서 키보드 입력 처리"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok() and self._running:
                key = self._get_key(timeout=0.1)
                if key is None:
                    continue

                if key == '\x03':  # Ctrl+C
                    break

                self._on_key_press(key)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ==================== UI ====================

    def _print_ui(self):
        """터미널 UI 출력"""
        mode_header = "MINI-ONLY MODE" if self.mini_only_mode else "QUIZ CONTROLLER"
        mini_only_controls = """
║    [1] Answer 1 → figure8 (DETECTING only)                   ║
║    [2] Answer 2 → vertical_a (DETECTING only)                ║""" if self.mini_only_mode else ""

        ui = f"""
╔══════════════════════════════════════════════════════════════╗
║                      {mode_header:^21}                       ║
╠══════════════════════════════════════════════════════════════╣
║  State Machine:                                              ║
║    UNINIT → IDLE → DETECTING → DRAWING → IDLE → ...         ║
║                  ↘      ↓         ↓      ↙                   ║
║                         FINISH                               ║
╠══════════════════════════════════════════════════════════════╣
║  Controls:                                                   ║
║    [s] Start (UnInit→Idle) / Return Home (Drawing→Idle)     ║
║    [n] Next - Start Detecting (Idle→Detecting)              ║
║    [x] Exit - Land and Finish                                ║
║    [SPACE] EMERGENCY STOP                                    ║{mini_only_controls}
║    Ctrl+C : Force quit                                       ║
╠══════════════════════════════════════════════════════════════╣
║  Topics:                                                     ║
║    Pub: /quiz/state, /quiz/answer (current state)            ║
║    Sub: /quiz/answer (detection result)                      ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(ui)

    def _update_ui(self):
        """상태 변경 시 UI 업데이트 (간단 로그)"""
        with self._lock:
            state_name = self.state.name
            timer_info = ""
            if self.operation_start_time is not None:
                elapsed = time.time() - self.operation_start_time
                remaining = max(0, self.operation_timeout - elapsed)
                timer_info = f" | Timer: {int(elapsed//60):02d}:{int(elapsed%60):02d} / {int(self.operation_timeout//60):02d}:00"

            pending_info = ""
            if self.pending_home_return:
                pending_info = " | Waiting for home arrival"
            elif self.pending_transition is not None:
                target, trans_time = self.pending_transition
                remaining = max(0, trans_time - time.time())
                pending_info = f" | Transitioning to {target.name} in {remaining:.1f}s"

        print(f"\n>>> State: [{state_name}]{timer_info}{pending_info}")

    def destroy_node(self):
        """노드 종료"""
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuizControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

