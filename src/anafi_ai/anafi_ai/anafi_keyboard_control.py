#!/usr/bin/env python3
# anafi_ai_moveby_keyboard.py
#
# Keyboard teleop for Parrot Anafi / Anafi AI using MoveByCommand
#
# - Publishes:
#     /anafi/drone/moveby       (anafi_ros_interfaces/MoveByCommand)
# - Subscribes:
#     /anafi/drone/moveby_done  (std_msgs/Bool)   ← 브리지에서 extended_move_by 결과 피드백
#     /anafi/camera/image       (sensor_msgs/Image, 기본값, 파라미터로 변경 가능)
# - Calls services:
#     /anafi/drone/takeoff      (std_srvs/Trigger)
#     /anafi/drone/land         (std_srvs/Trigger)
#     /anafi/drone/rth          (std_srvs/Trigger)
#     /anafi/drone/halt         (std_srvs/Trigger)
#     /anafi/skycontroller/offboard (std_srvs/SetBool, 옵션)
#
# MoveBy 축 정의 (bridge 코드 주석 기준):
#   d_x  : front (+) / back (−)
#   d_y  : right (+) / left (−)  ← 하지만 여기서는 오른쪽 이동을 dy<0 로 맞춰 사용
#   d_z  : down (+) / up (−)
#   d_psi: heading rotation [rad] (양수=CCW)
#
# 키 매핑 (한 번 누를 때마다 한 step 움직임):
#   w/s : 전/후 (±dx)
#   a/d : 좌/우 (±dy, d=오른쪽)
#   r/f : 상/하 (dz=∓step, r=위로)
#   q/e : 좌/우 yaw (±d_psi)
#   대문자(WASDQERFQ E): 터보 스텝 (기본의 turbo_multiplier 배)
#
#   t : 이륙
#   l : 착륙
#   h : RTH
#   k : HALT (강제 정지)
#   o : skycontroller/offboard 토글
#   z/x : linear/yaw step 크기 ↓/↑
#   ? : 도움말

import sys
import termios
import tty
import select
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from anafi_ros_interfaces.msg import MoveByCommand
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import json
import math


HELP_TEXT = r"""
Anafi AI MoveBy Keyboard Controller

이동 (MoveBy, 한 번 누를 때마다 한 스텝)
  w/s : +dx / -dx   (전/후)
  a/d : +dy / -dy   (좌/우, d=오른쪽)
  r/f : 위 / 아래  (dz = -/+ step, up/down)
  q/e : +yaw / -yaw (좌/우 회전, deg → rad)

스텝 크기
  기본 linear step : lin_step (m)  [기본 0.5 m]
  기본 yaw step    : yaw_step (deg) [기본 15 deg]
  대문자 입력(WASDQERFQE) : turbo_multiplier 배 (기본 2.0배)
  z/x : linear/yaw step 축소/확대

동작
  t : 이륙 (/anafi/drone/takeoff)
  l : 착륙 (/anafi/drone/land)
  h : 귀환 (/anafi/drone/rth)
  k : HALT 강제 정지 (/anafi/drone/halt)
  o : offboard 토글 (/anafi/skycontroller/offboard)
  c : 화면 추적 모드 토글 (YOLO 타겟을 중앙으로 이동)
  1 : 성공 모션 (위아래 흔들기 + 회전)
  2 : 실패 모션 (좌우 흔들기)
  ? : 이 도움말 출력

카메라
  /anafi/<camera_topic> (기본: /anafi/camera/image)를 subscribe 해서
  프레임 수신 여부와 간단한 정보만 로그로 남깁니다.

Ctrl+C 로 종료
"""


def _make_qos(depth=10, reliable=True):
    q = QoSProfile(
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
    )
    return q


class _Keyboard:
    """터미널 비차단 단일 문자 입력 헬퍼"""

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


class AnafiMoveByKeyboard(Node):
    def __init__(self):
        # /anafi 네임스페이스에 고정
        super().__init__('anafi_moveby_keyboard', namespace='/anafi')

        # ---------- 파라미터 ----------
        # 스텝 크기
        self.declare_parameter('lin_step', 0.5)          # 기본 linear step [m]
        self.declare_parameter('yaw_step_deg', 15.0)     # 기본 yaw step [deg]
        self.declare_parameter('turbo_multiplier', 2.0)  # 대문자 입력시 배수

        # 카메라 토픽 (namespace 상대 경로)
        self.declare_parameter('camera_topic', 'camera/image')

        # 시작 시 offboard ON 여부
        self.declare_parameter('enable_offboard_on_start', True)

        self.lin_step = float(self.get_parameter('lin_step').value)
        self.yaw_step_deg = float(self.get_parameter('yaw_step_deg').value)
        self.turbo_multiplier = float(self.get_parameter('turbo_multiplier').value)
        self.camera_topic = self.get_parameter('camera_topic').value

        # ---------- 토픽/서비스 ----------
        qos_ctrl = _make_qos(depth=10, reliable=True)
        qos_cam = _make_qos(depth=5, reliable=False)

        # MoveBy pub/sub
        self.pub_moveby = self.create_publisher(MoveByCommand, 'drone/moveby', qos_ctrl)
        self.sub_moveby_done = self.create_subscription(
            Bool, 'drone/moveby_done', self._on_moveby_done, qos_ctrl
        )

        # 카메라
        self.sub_camera = self.create_subscription(
            Image, self.camera_topic, self._on_camera, qos_cam
        )

        # 서비스
        self.cli_takeoff = self.create_client(Trigger, 'drone/takeoff')
        self.cli_land = self.create_client(Trigger, 'drone/land')
        self.cli_rth = self.create_client(Trigger, 'drone/rth')
        self.cli_halt = self.create_client(Trigger, 'drone/halt')
        self.cli_offboard = self.create_client(SetBool, 'skycontroller/offboard')

        # 카메라 상태
        self._camera_frame_count = 0
        
        # ---------- Tracking State ----------
        self._tracking_enabled = False
        self._tracking_status = None  # Latest tracking status from YOLO
        self._tracking_move_scale = 0.8  # Meters per normalized offset (increased for actual movement)
        self._tracking_min_move = 0.15    # Minimum movement threshold (m) - Anafi may ignore smaller moves
        self._tracking_max_move = 0.25    # Maximum movement per step (m)
        self._tracking_timer = None       # Timer for auto tracking
        self._tracking_interval = 0.5     # Seconds between tracking checks (faster polling)
        self._tracking_move_pending = False  # Flag to wait for move completion
        self._tracking_stabilize_time = 1.0  # Seconds to wait after move completion for image stabilization
        self._tracking_move_done_time = None  # Time when last move completed
        
        # Bbox size target for forward/backward control (dx)
        self._target_bbox_width = 270.0   # Target bbox width in pixels
        self._target_bbox_height = 200.0  # Target bbox height in pixels
        self._bbox_tolerance = 0.15       # 10% tolerance
        self._tracking_dx_step = 0.15     # Forward/backward step size (m)
        self._tracking_centered_once = False  # True if centered at least once (then only dx)
        
        # Position tracking for movement verification
        self._current_position = None     # Current drone position (x, y, z)
        self._position_before_move = None # Position before moveby command
        self._move_distance_threshold = 0.05  # Minimum distance (m) to consider "moved"
        self._no_move_count = 0           # Count of consecutive no-movement detections
        self._no_move_threshold = 3       # After this many no-moves, consider centered
        
        # Motion state
        self._motion_in_progress = False
        self._motion_sequence = []        # List of (dx, dy, dz, dyaw) tuples
        self._motion_index = 0            # Current index in motion sequence
        self._motion_timer = None         # Timer for motion sequence
        
        # Tracking enable publisher
        self.pub_tracking_enable = self.create_publisher(
            Bool, 'yolo/tracking_enable', qos_ctrl
        )
        
        # OCR enable publisher
        self._ocr_enabled = False
        self.pub_ocr_enable = self.create_publisher(
            Bool, 'yolo/ocr_enable', qos_ctrl
        )
        
        # Tracking status subscriber
        self.sub_tracking_status = self.create_subscription(
            String, 'yolo/tracking_status', self._on_tracking_status, qos_ctrl
        )
        
        # Position subscriber for movement verification
        qos_sensor = _make_qos(depth=10, reliable=False)  # BEST_EFFORT for sensor data
        self.sub_position = self.create_subscription(
            PointStamped, 'drone/position_local', self._on_position, qos_sensor
        )

        # 키보드 초기화
        self._kb = None
        try:
            self._kb = _Keyboard()
        except Exception as e:
            self.get_logger().warn(f"키보드 초기화 실패 (tty 필요): {e!r}")

        # 키보드용 타이머 (스레드 대신 타이머로 폴링)
        if self._kb:
            self.timer_kb = self.create_timer(0.01, self._keyboard_tick)
        else:
            self.get_logger().warn("키보드 입력이 없으므로 텔레옵은 동작하지 않습니다.")

        # 로그
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f"MoveBy 토픽: /anafi/drone/moveby")
        self.get_logger().info(f"MoveBy 완료 토픽: /anafi/drone/moveby_done")
        self.get_logger().info(f"카메라 토픽: /anafi/{self.camera_topic}")
        self.get_logger().info(
            f"초기 스텝: lin_step={self.lin_step:.2f} m, yaw_step={self.yaw_step_deg:.1f} deg"
        )

        # 시작 시 offboard ON
        if bool(self.get_parameter('enable_offboard_on_start').value):
            self._set_offboard(True)

    # ---------- 카메라 콜백 ----------
    def _on_camera(self, msg: Image):
        self._camera_frame_count += 1
        if self._camera_frame_count == 1:
            self.get_logger().info(
                f"카메라 첫 프레임 수신: {msg.width}x{msg.height}, encoding={msg.encoding}"
            )
        elif self._camera_frame_count % 100 == 0:
            self.get_logger().debug(
                f"카메라 프레임 {self._camera_frame_count}개 수신 중..."
            )

    # ---------- Position 콜백 ----------
    def _on_position(self, msg: PointStamped):
        """Receive local position from drone."""
        self._current_position = (msg.point.x, msg.point.y, msg.point.z)

    # ---------- MoveBy 완료 콜백 ----------
    def _on_moveby_done(self, msg: Bool):
        # 모션 시퀀스 진행 중이면 다음 동작 실행
        if self._motion_in_progress:
            # 짧은 딜레이 후 다음 동작 실행 (0.1초)
            if self._motion_timer is not None:
                self._motion_timer.cancel()
            self._motion_timer = self.create_timer(0.05, self._motion_timer_callback)
        
        # Clear tracking move pending flag and record completion time
        if self._tracking_move_pending:
            self._tracking_move_pending = False
            self._tracking_move_done_time = time.time()  # Record when move completed
            
            # Check if drone actually moved
            if self._position_before_move is not None and self._current_position is not None:
                dx = self._current_position[0] - self._position_before_move[0]
                dy = self._current_position[1] - self._position_before_move[1]
                dz = self._current_position[2] - self._position_before_move[2]
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance < self._move_distance_threshold:
                    self._no_move_count += 1
                    self.get_logger().warn(
                        f"[Tracking] 이동 명령 완료했지만 실제 이동 없음! "
                        f"(거리: {distance:.3f}m < {self._move_distance_threshold}m, "
                        f"연속 {self._no_move_count}회)"
                    )
                    
                    # If no movement detected multiple times, consider it centered
                    if self._no_move_count >= self._no_move_threshold and not self._tracking_centered_once:
                        self._tracking_centered_once = True
                        self.get_logger().warning(
                            f"[Tracking] ★ {self._no_move_threshold}회 연속 이동 없음 → 좌우/상하 정렬 완료로 간주! ★"
                        )
                else:
                    self._no_move_count = 0  # Reset counter on successful move
                    self.get_logger().info(
                        f"[Tracking] 이동 완료 - 실제 이동 거리: {distance:.3f}m"
                    )
            
            self._position_before_move = None  # Clear saved position
            
            if self._tracking_enabled:
                self.get_logger().info("[Tracking] 이동 완료 - 안정화 대기 시작")
        
        if msg.data:
            self.get_logger().info("MoveBy 완료 ✅")
        else:
            self.get_logger().warn("MoveBy 실패 ❌")

    def _motion_timer_callback(self):
        """모션 타이머 콜백 - 다음 동작 실행"""
        if self._motion_timer is not None:
            self._motion_timer.cancel()
            self._motion_timer = None
        self._execute_next_motion()

    # ---------- Tracking 콜백 ----------
    def _on_tracking_status(self, msg: String):
        """Receive tracking status from YOLO detection node."""
        try:
            self._tracking_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse tracking status JSON")

    def _toggle_tracking(self):
        """Toggle tracking mode on/off."""
        self._tracking_enabled = not self._tracking_enabled
        
        # Publish tracking enable state
        msg = Bool()
        msg.data = self._tracking_enabled
        self.pub_tracking_enable.publish(msg)
        
        if self._tracking_enabled:
            self.get_logger().warning("[Tracking] 추적 모드 ON - 자동으로 타겟을 중앙으로 이동합니다")
            self._tracking_centered_once = False  # Reset centered flag
            self._no_move_count = 0  # Reset no-move counter
            self._start_tracking_timer()
        else:
            self.get_logger().warning("[Tracking] 추적 모드 OFF")
            self._stop_tracking_timer()
            self._tracking_status = None
            self._tracking_centered_once = False  # Reset centered flag
            self._no_move_count = 0  # Reset no-move counter

    def _toggle_ocr(self):
        """Toggle OCR mode on/off."""
        self._ocr_enabled = not self._ocr_enabled
        
        # Publish OCR enable state
        msg = Bool()
        msg.data = self._ocr_enabled
        self.pub_ocr_enable.publish(msg)
        
        if self._ocr_enabled:
            self.get_logger().warning("[OCR] OCR 모드 ON - 수식 인식을 시작합니다")
        else:
            self.get_logger().warning("[OCR] OCR 모드 OFF")

    # ---------- 성공/실패 모션 ----------
    def _start_success_motion(self):
        """성공 모션: 위아래로 빠르게 왕복 2번"""
        if self._motion_in_progress:
            self.get_logger().warn("[Motion] 이미 모션 진행 중입니다")
            return
        
        self.get_logger().warning("[Motion] 🎉 성공! 위아래 왕복 2번 🎉")
        
        # 위아래 왕복 2번 (빠르게)
        step = 0.2  # 20cm 위아래
        
        self._motion_sequence = [
            # 왕복 1
            (0.0, 0.0, -step, 0.0),  # 위로
            (0.0, 0.0, step, 0.0),   # 아래로 (원위치)
            # 왕복 2
            (0.0, 0.0, -step, 0.0),  # 위로
            (0.0, 0.0, step, 0.0),   # 아래로 (원위치)
        ]
        
        self._motion_index = 0
        self._motion_in_progress = True
        self._execute_next_motion()

    def _start_failure_motion(self):
        """실패 모션: 좌우로 빠르게 회전 왕복 2번"""
        if self._motion_in_progress:
            self.get_logger().warn("[Motion] 이미 모션 진행 중입니다")
            return
        
        self.get_logger().warning("[Motion] 😢 실패! 좌우 회전 왕복 2번 😢")
        
        # 좌우 회전 왕복 2번 (빠르게)
        yaw_deg = 30.0  # 30도 회전
        yaw_rad = yaw_deg * 3.141592653589793 / 180.0
        
        self._motion_sequence = [
            # 왕복 1
            (0.0, 0.0, 0.0, -yaw_rad),  # 왼쪽 회전
            (0.0, 0.0, 0.0, yaw_rad),   # 오른쪽 회전 (원위치)
            # 왕복 2
            (0.0, 0.0, 0.0, yaw_rad),  # 왼쪽 회전
            (0.0, 0.0, 0.0, -yaw_rad),   # 오른쪽 회전 (원위치)
        ]
        
        self._motion_index = 0
        self._motion_in_progress = True
        self._execute_next_motion()

    def _execute_next_motion(self):
        """모션 시퀀스의 다음 동작 실행"""
        if self._motion_index >= len(self._motion_sequence):
            # 모션 완료
            self._motion_in_progress = False
            self._motion_sequence = []
            self._motion_index = 0
            self.get_logger().warning("[Motion] ✅ 모션 완료!")
            return
        
        dx, dy, dz, dyaw = self._motion_sequence[self._motion_index]
        self.get_logger().info(
            f"[Motion] 동작 {self._motion_index + 1}/{len(self._motion_sequence)}: "
            f"dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.2f}"
        )
        self._publish_moveby(dx=dx, dy=dy, dz=dz, dyaw=dyaw)
        self._motion_index += 1

    def _start_tracking_timer(self):
        """Start the auto-tracking timer."""
        if self._tracking_timer is not None:
            self._tracking_timer.cancel()
        self._tracking_timer = self.create_timer(
            self._tracking_interval, self._tracking_timer_callback
        )
        self.get_logger().info(f"[Tracking] 자동 추적 타이머 시작 (간격: {self._tracking_interval}s)")

    def _stop_tracking_timer(self):
        """Stop the auto-tracking timer."""
        if self._tracking_timer is not None:
            self._tracking_timer.cancel()
            self._tracking_timer = None
            self.get_logger().info("[Tracking] 자동 추적 타이머 중지")

    def _tracking_timer_callback(self):
        """Timer callback for auto-tracking movement."""
        import time
        
        if not self._tracking_enabled:
            self._stop_tracking_timer()
            return
        
        # Wait for previous move to complete
        if self._tracking_move_pending:
            self.get_logger().debug("[Tracking] 이전 이동 완료 대기 중...")
            return
        
        # Wait for stabilization after move completion
        if self._tracking_move_done_time is not None:
            elapsed = time.time() - self._tracking_move_done_time
            if elapsed < self._tracking_stabilize_time:
                self.get_logger().debug(f"[Tracking] 안정화 대기 중... ({elapsed:.1f}/{self._tracking_stabilize_time}s)")
                return
            else:
                # Stabilization complete, clear the flag
                self._tracking_move_done_time = None
                self.get_logger().info("[Tracking] 안정화 완료 - 새 위치 확인")
        
        if self._tracking_status is None:
            self.get_logger().debug("[Tracking] 추적 상태 대기 중...")
            return
        
        status = self._tracking_status
        
        if not status.get('detected', False):
            self.get_logger().debug("[Tracking] 타겟 미감지 - 대기 중")
            return
        
        if status.get('centered', False):
            self.get_logger().info("[Tracking] ★ 타겟 중앙 정렬 완료! 추적 모드 자동 종료 ★")
            self._toggle_tracking()
            return
        
        # Execute tracking move
        self._execute_tracking_move()

    def _execute_tracking_move(self):
        """
        Execute a single tracking movement based on latest tracking status.
        Called when user presses 'c' while tracking is enabled.
        
        Movement mapping (camera view to drone movement):
        - offset_x > 0 (target is right) -> dy > 0 (move right)
        - offset_y > 0 (target is below) -> dz > 0 (move down)
        """
        if self._tracking_status is None:
            self.get_logger().warn("[Tracking] 추적 상태 없음 - YOLO 노드 확인 필요")
            return
        
        status = self._tracking_status
        
        if not status.get('tracking_enabled', False):
            self.get_logger().warn("[Tracking] 추적 모드가 YOLO에서 비활성화됨")
            return
        
        if not status.get('detected', False):
            self.get_logger().warn("[Tracking] 타겟이 감지되지 않음")
            return
        
        # Get bbox info
        bbox_width = status.get('bbox_width', 0.0)
        bbox_height = status.get('bbox_height', 0.0)
        bbox_area = status.get('bbox_area', 0.0)
        bbox_aspect_ratio = status.get('bbox_aspect_ratio', 1.0)
        
        # Get normalized offsets (-1 to 1)
        offset_x_norm = status.get('offset_x_normalized', 0.0)
        offset_y_norm = status.get('offset_y_normalized', 0.0)
        
        # Log bbox info
        self.get_logger().info(
            f"[Tracking] bbox: {bbox_width:.0f}x{bbox_height:.0f}, "
            f"area={bbox_area:.0f}, ratio={bbox_aspect_ratio:.2f}"
        )
        
        # Phase 1: 먼저 좌우/상하 정렬 (dy, dz) - 한번 centered 되면 스킵
        if not self._tracking_centered_once:
            # centered는 dy==0, dz==0으로 판단
            # Calculate movement (scale normalized offset to meters)
            # 드론 좌표계: dy=좌우, dz=상하
            dy = offset_x_norm * self._tracking_move_scale  # 좌우 (카메라 X offset)
            dz = offset_y_norm * self._tracking_move_scale  # 상하 (카메라 Y offset)
            
            # Apply minimum threshold
            if abs(dy) < self._tracking_min_move:
                dy = 0.0
            if abs(dz) < self._tracking_min_move:
                dz = 0.0
            
            # Clamp to maximum
            dy = max(-self._tracking_max_move, min(self._tracking_max_move, dy))
            dz = max(-self._tracking_max_move, min(self._tracking_max_move, dz))
            
            # dy, dz가 0이 아니면 아직 정렬 중
            if dy != 0.0 or dz != 0.0:
                self.get_logger().info(
                    f"[Tracking] 정렬 이동: dy={dy:.3f}m(좌우), dz={dz:.3f}m(상하) "
                    f"(offset: x={offset_x_norm:.2f}, y={offset_y_norm:.2f})"
                )
                self._tracking_move_pending = True
                self._publish_moveby(dx=0.0, dy=dy, dz=dz)  # dx=0 명시
                return
            
            # dy==0, dz==0 이면 centered - 플래그 설정
            self._tracking_centered_once = True
            self.get_logger().info("[Tracking] ★ 좌우/상하 정렬 완료! 이제 앞뒤(dx)만 조절합니다 ★")
        
        # Phase 2: 중앙 정렬 완료 후 bbox 크기로 앞뒤(dx) 조절
        # 목표: 270x200 ± 10%
        width_min = self._target_bbox_width * (1.0 - self._bbox_tolerance)
        width_max = self._target_bbox_width * (1.0 + self._bbox_tolerance)
        height_min = self._target_bbox_height * (1.0 - self._bbox_tolerance)
        height_max = self._target_bbox_height * (1.0 + self._bbox_tolerance)
        
        width_ok = width_min <= bbox_width <= width_max
        height_ok = height_min <= bbox_height <= height_max
        
        if width_ok and height_ok:
            # 모든 조건 충족 - 추적 완료!
            self.get_logger().info(
                f"[Tracking] ★ 완료! bbox={bbox_width:.0f}x{bbox_height:.0f} "
                f"(목표: {self._target_bbox_width:.0f}x{self._target_bbox_height:.0f} ±{self._bbox_tolerance*100:.0f}%) ★"
            )
            self._toggle_tracking()
            
            # 추적 완료 시 자동으로 OCR 활성화
            if not self._ocr_enabled:
                self._ocr_enabled = True
                msg = Bool()
                msg.data = True
                self.pub_ocr_enable.publish(msg)
                self.get_logger().warning("[OCR] 추적 완료 → OCR 자동 활성화!")
            return
        
        # bbox가 너무 작으면 앞으로(+dx), 너무 크면 뒤로(-dx)
        dx = 0.0
        if bbox_width < width_min or bbox_height < height_min:
            dx = self._tracking_dx_step  # 앞으로 (가까이)
            self.get_logger().info(
                f"[Tracking] bbox 작음 → 앞으로 이동: dx={dx:.3f}m "
                f"(현재: {bbox_width:.0f}x{bbox_height:.0f}, 목표: {width_min:.0f}~{width_max:.0f})"
            )
        elif bbox_width > width_max or bbox_height > height_max:
            dx = -self._tracking_dx_step  # 뒤로 (멀리)
            self.get_logger().info(
                f"[Tracking] bbox 큼 → 뒤로 이동: dx={dx:.3f}m "
                f"(현재: {bbox_width:.0f}x{bbox_height:.0f}, 목표: {width_min:.0f}~{width_max:.0f})"
            )
        
        if dx != 0.0:
            self._tracking_move_pending = True
            self._publish_moveby(dx=dx, dy=0.0, dz=0.0)

    # ---------- 서비스 헬퍼 ----------
    def _call_trigger_async(self, client, name: str):
        if not client.service_is_ready():
            self.get_logger().info(f"{name} 서비스 대기 중...")
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f"{name} 서비스 준비 안 됨")
                return
        fut = client.call_async(Trigger.Request())

        def _done(_):
            try:
                resp = fut.result()
                if resp is None:
                    self.get_logger().warn(f"{name} 응답 없음")
                    return
                if resp.success:
                    self.get_logger().info(f"{name}: {resp.message}")
                else:
                    self.get_logger().warn(f"{name} 실패: {resp.message}")
            except Exception as e:
                self.get_logger().error(f"{name} 호출 오류: {e!r}")

        fut.add_done_callback(_done)

    def _set_offboard(self, enable: bool):
        if not self.cli_offboard.service_is_ready():
            self.get_logger().info("offboard 서비스 대기 중...")
            if not self.cli_offboard.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn("offboard 서비스 준비 안 됨")
                return

        from std_srvs.srv import SetBool  # 재확인용

        req = SetBool.Request()
        req.data = enable
        fut = self.cli_offboard.call_async(req)

        def _done(_):
            try:
                resp = fut.result()
                if resp is None:
                    self.get_logger().warn("offboard 응답 없음")
                    return
                state = "ON" if (enable and resp.success) else "OFF"
                self.get_logger().warning(f"Offboard → {state} ({resp.message})")
            except Exception as e:
                self.get_logger().error(f"offboard 설정 오류: {e!r}")

        fut.add_done_callback(_done)

    # ---------- MoveBy 발행 ----------
    def _publish_moveby(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        # Save current position before move for verification
        if self._current_position is not None:
            self._position_before_move = self._current_position
        
        msg = MoveByCommand()
        msg.dx = float(dx)
        msg.dy = float(dy)
        msg.dz = float(dz)
        msg.dyaw = float(dyaw)
        self.pub_moveby.publish(msg)
        self.get_logger().info(
            f"MoveBy PUB → dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.3f} rad"
        )

    # ---------- 키보드 폴링 ----------
    def _keyboard_tick(self):
        ch = self._kb.getch()
        if ch is None:
            return

        # 추적 모드 중 다른 키가 눌리면 즉시 중지 (c/C 제외)
        if self._tracking_enabled and ch not in ('c', 'C'):
            self.get_logger().warning(f"[Tracking] 키 '{ch}' 입력 - 추적 모드 중지")
            self._tracking_enabled = False
            self._stop_tracking_timer()
            # YOLO에도 추적 비활성화 알림
            msg = Bool()
            msg.data = False
            self.pub_tracking_enable.publish(msg)
            self._tracking_status = None
            # 키 입력은 계속 처리

        turbo = ch.isupper()
        lin = self.lin_step * (self.turbo_multiplier if turbo else 1.0)
        yaw_deg = self.yaw_step_deg * (self.turbo_multiplier if turbo else 1.0)
        yaw_rad = yaw_deg * 3.141592653589793 / 180.0

        # --- 이동 키 (MoveBy) ---
        if ch in ('w', 'W'):
            # 전진 (+dx)
            self._publish_moveby(dx=lin)
        elif ch in ('s', 'S'):
            # 후진 (-dx)
            self._publish_moveby(dx=-lin)
        elif ch in ('d', 'D'):
            # 오른쪽 (dy>0 로 사용, TestFlight 예제에 맞춰)
            self._publish_moveby(dy=lin)
        elif ch in ('a', 'A'):
            # 왼쪽 (dy<0)
            self._publish_moveby(dy=-lin)
        elif ch in ('r', 'R'):
            # 위로 (dz<0, down axis 기준)
            self._publish_moveby(dz=-lin)
        elif ch in ('f', 'F'):
            # 아래로 (dz>0)
            self._publish_moveby(dz=lin)
        elif ch in ('q', 'Q'):
            # 좌회전 (+yaw)
            self._publish_moveby(dyaw=yaw_rad)
        elif ch in ('e', 'E'):
            # 우회전 (-yaw)
            self._publish_moveby(dyaw=-yaw_rad)

        # --- 스텝 크기 조절 ---
        elif ch == 'z':
            self.lin_step = max(0.1, self.lin_step * 0.8)
            self.yaw_step_deg = max(1.0, self.yaw_step_deg * 0.8)
            self.get_logger().info(
                f"스텝 축소: lin_step={self.lin_step:.2f} m, yaw_step={self.yaw_step_deg:.1f} deg"
            )
        elif ch == 'x':
            self.lin_step = min(5.0, self.lin_step * 1.25)
            self.yaw_step_deg = min(90.0, self.yaw_step_deg * 1.25)
            self.get_logger().info(
                f"스텝 확대: lin_step={self.lin_step:.2f} m, yaw_step={self.yaw_step_deg:.1f} deg"
            )

        # --- 드론 동작 키 ---
        elif ch == 't':
            self.get_logger().warning("[키보드] 이륙 요청 (t)")
            self._call_trigger_async(self.cli_takeoff, 'takeoff')
        elif ch == 'l':
            self.get_logger().warning("[키보드] 착륙 요청 (l)")
            self._call_trigger_async(self.cli_land, 'land')
        elif ch == 'h':
            self.get_logger().warning("[키보드] RTH 요청 (h)")
            self._call_trigger_async(self.cli_rth, 'rth')
        elif ch == 'k':
            self.get_logger().warning("[키보드] HALT 강제 정지 요청 (k)")
            self._call_trigger_async(self.cli_halt, 'halt')
        elif ch == 'o':
            self.get_logger().warning("[키보드] offboard 토글 요청 (o)")
            # 간단하게 토글만, 실제 상태는 응답 로그로 확인
            self._set_offboard(True)  # 필요하면 토글 상태를 멤버로 들고가도 됨
        
        # --- 추적 모드 ---
        elif ch in ('c', 'C'):
            # c 또는 C: 추적 모드 토글
            self._toggle_tracking()
        
        # --- OCR 모드 ---
        elif ch in ('n', 'N'):
            # n 또는 N: OCR 토글
            self._toggle_ocr()
        
        # --- 성공/실패 모션 ---
        elif ch == '1':
            # 1: 성공 모션
            self._start_success_motion()
        elif ch == '2':
            # 2: 실패 모션
            self._start_failure_motion()
        
        elif ch == '?':
            self.get_logger().info(HELP_TEXT)
        # 그 외 키는 무시


def main():
    rclpy.init()
    node = AnafiMoveByKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "_kb") and node._kb:
            try:
                node._kb.restore()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
