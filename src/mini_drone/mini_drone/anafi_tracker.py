#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ANAFI Tracker Module

YOLO detection 결과를 기반으로 ANAFI 드론을 자동으로 타겟 중앙에 위치시키는 모듈.

Features:
    - Phase 1: 좌우/상하 정렬 (dy, dz)
    - Phase 2: 앞뒤 거리 조절 (dx) - bbox 크기 기반
    - 이동 완료 후 안정화 대기
    - 실제 이동 검증 (position feedback)

Topics:
    Subscribe:
        - /anafi/yolo/tracking_status (String, JSON)
        - /anafi/drone/moveby_done (Bool)
        - /anafi/drone/position_local (PointStamped)
    
    Publish:
        - /anafi/yolo/tracking_enable (Bool)
        - /anafi/drone/moveby (MoveByCommand)
"""

import json
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from anafi_ros_interfaces.msg import MoveByCommand


def _make_qos(depth=10, reliable=True):
    return QoSProfile(
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        history=HistoryPolicy.KEEP_LAST,
    )


class TrackingPhase(Enum):
    """Tracking 단계"""
    IDLE = auto()           # 추적 비활성화
    CENTERING = auto()      # Phase 1: 좌우/상하 정렬
    DISTANCE_ADJUST = auto()  # Phase 2: 앞뒤 거리 조절
    COMPLETED = auto()      # 추적 완료


@dataclass
class TrackingConfig:
    """Tracking 설정"""
    # 이동 스케일
    move_scale: float = 0.8         # Meters per normalized offset
    min_move: float = 0.15          # Minimum movement threshold (m)
    max_move: float = 0.25          # Maximum movement per step (m)
    
    # 타이머 설정
    tracking_interval: float = 0.5  # Seconds between tracking checks
    stabilize_time: float = 1.0     # Seconds to wait after move completion
    
    # Bbox 목표 크기
    target_bbox_width: float = 270.0    # Target bbox width in pixels
    target_bbox_height: float = 200.0   # Target bbox height in pixels
    bbox_tolerance: float = 0.15        # 15% tolerance
    dx_step: float = 0.15               # Forward/backward step size (m)
    
    # 이동 검증
    move_distance_threshold: float = 0.05  # Minimum distance (m) to consider "moved"
    no_move_threshold: int = 3             # After this many no-moves, consider centered


class AnafiTracker:
    """
    ANAFI 드론 자동 추적 모듈
    
    YOLO detection 결과를 받아서 드론을 타겟 중앙에 위치시킵니다.
    """
    
    def __init__(
        self,
        node: Node,
        config: Optional[TrackingConfig] = None,
        on_tracking_complete: Optional[Callable[[], None]] = None,
        namespace: str = '/anafi'
    ):
        """
        Args:
            node: ROS2 Node 인스턴스
            config: Tracking 설정
            on_tracking_complete: 추적 완료 시 호출될 콜백
            namespace: ANAFI 토픽 네임스페이스
        """
        self._node = node
        self._config = config or TrackingConfig()
        self._on_complete = on_tracking_complete
        self._namespace = namespace
        self._logger = node.get_logger()
        
        # ---- State ----
        self._enabled = False
        self._phase = TrackingPhase.IDLE
        self._tracking_status: Optional[dict] = None
        
        # 이동 상태
        self._move_pending = False
        self._move_done_time: Optional[float] = None
        
        # 위치 추적
        self._current_position: Optional[tuple] = None
        self._position_before_move: Optional[tuple] = None
        self._no_move_count = 0
        
        # Timer
        self._tracking_timer = None
        
        # ---- QoS ----
        qos_ctrl = _make_qos(depth=10, reliable=True)
        qos_sensor = _make_qos(depth=10, reliable=False)
        
        # ---- Publishers ----
        self.pub_tracking_enable = node.create_publisher(
            Bool, f'{namespace}/yolo/tracking_enable', qos_ctrl
        )
        self.pub_moveby = node.create_publisher(
            MoveByCommand, f'{namespace}/drone/moveby', qos_ctrl
        )
        
        # ---- Subscribers ----
        self.sub_tracking_status = node.create_subscription(
            String, f'{namespace}/yolo/tracking_status',
            self._on_tracking_status, qos_ctrl
        )
        self.sub_moveby_done = node.create_subscription(
            Bool, f'{namespace}/drone/moveby_done',
            self._on_moveby_done, qos_ctrl
        )
        self.sub_position = node.create_subscription(
            PointStamped, f'{namespace}/drone/position_local',
            self._on_position, qos_sensor
        )
        
        self._logger.info("[AnafiTracker] Initialized")
    
    # ==================== Public API ====================
    
    def start_tracking(self):
        """추적 시작"""
        if self._enabled:
            self._logger.warn("[AnafiTracker] Already tracking")
            return
        
        self._enabled = True
        self._phase = TrackingPhase.CENTERING
        self._no_move_count = 0
        self._tracking_status = None
        
        # YOLO에 추적 활성화 알림
        msg = Bool()
        msg.data = True
        self.pub_tracking_enable.publish(msg)
        
        # 타이머 시작
        self._start_timer()
        
        self._logger.warning("[AnafiTracker] ★ Tracking STARTED - Phase 1: Centering ★")
    
    def stop_tracking(self):
        """추적 중지"""
        if not self._enabled:
            return
        
        self._enabled = False
        self._phase = TrackingPhase.IDLE
        
        # YOLO에 추적 비활성화 알림
        msg = Bool()
        msg.data = False
        self.pub_tracking_enable.publish(msg)
        
        # 타이머 중지
        self._stop_timer()
        
        # 상태 초기화
        self._tracking_status = None
        self._move_pending = False
        self._move_done_time = None
        self._no_move_count = 0
        
        self._logger.warning("[AnafiTracker] Tracking STOPPED")
    
    @property
    def is_tracking(self) -> bool:
        """추적 활성화 여부"""
        return self._enabled
    
    @property
    def phase(self) -> TrackingPhase:
        """현재 추적 단계"""
        return self._phase
    
    # ==================== Callbacks ====================
    
    def _on_tracking_status(self, msg: String):
        """YOLO로부터 tracking status 수신"""
        try:
            self._tracking_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._logger.warn("[AnafiTracker] Failed to parse tracking status JSON")
    
    def _on_moveby_done(self, msg: Bool):
        """MoveBy 명령 완료 콜백"""
        if not self._move_pending:
            return
        
        self._move_pending = False
        self._move_done_time = time.time()
        
        # 실제 이동 검증
        if self._position_before_move is not None and self._current_position is not None:
            dx = self._current_position[0] - self._position_before_move[0]
            dy = self._current_position[1] - self._position_before_move[1]
            dz = self._current_position[2] - self._position_before_move[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if distance < self._config.move_distance_threshold:
                self._no_move_count += 1
                self._logger.warn(
                    f"[AnafiTracker] Move completed but no actual movement! "
                    f"(dist: {distance:.3f}m < {self._config.move_distance_threshold}m, "
                    f"consecutive: {self._no_move_count})"
                )
                
                # 여러 번 이동 없으면 centered로 간주
                if self._no_move_count >= self._config.no_move_threshold:
                    if self._phase == TrackingPhase.CENTERING:
                        self._phase = TrackingPhase.DISTANCE_ADJUST
                        self._no_move_count = 0
                        self._logger.warning(
                            f"[AnafiTracker] ★ {self._config.no_move_threshold} consecutive no-moves → "
                            f"Centering COMPLETE! Moving to Phase 2: Distance Adjust ★"
                        )
            else:
                self._no_move_count = 0
                self._logger.info(f"[AnafiTracker] Move completed - actual distance: {distance:.3f}m")
        
        self._position_before_move = None
        
        if msg.data:
            self._logger.debug("[AnafiTracker] MoveBy completed ✅")
        else:
            self._logger.warn("[AnafiTracker] MoveBy failed ❌")
    
    def _on_position(self, msg: PointStamped):
        """드론 위치 수신"""
        self._current_position = (msg.point.x, msg.point.y, msg.point.z)
    
    # ==================== Timer ====================
    
    def _start_timer(self):
        """추적 타이머 시작"""
        if self._tracking_timer is not None:
            self._tracking_timer.cancel()
        
        self._tracking_timer = self._node.create_timer(
            self._config.tracking_interval,
            self._timer_callback
        )
        self._logger.info(f"[AnafiTracker] Timer started (interval: {self._config.tracking_interval}s)")
    
    def _stop_timer(self):
        """추적 타이머 중지"""
        if self._tracking_timer is not None:
            self._tracking_timer.cancel()
            self._tracking_timer = None
            self._logger.info("[AnafiTracker] Timer stopped")
    
    def _timer_callback(self):
        """주기적 추적 로직"""
        if not self._enabled:
            self._stop_timer()
            return
        
        # 이전 이동 완료 대기
        if self._move_pending:
            self._logger.debug("[AnafiTracker] Waiting for previous move to complete...")
            return
        
        # 안정화 대기
        if self._move_done_time is not None:
            elapsed = time.time() - self._move_done_time
            if elapsed < self._config.stabilize_time:
                self._logger.debug(
                    f"[AnafiTracker] Stabilizing... ({elapsed:.1f}/{self._config.stabilize_time}s)"
                )
                return
            else:
                self._move_done_time = None
                self._logger.info("[AnafiTracker] Stabilization complete")
        
        # 추적 상태 확인
        if self._tracking_status is None:
            self._logger.debug("[AnafiTracker] Waiting for tracking status...")
            return
        
        status = self._tracking_status
        
        if not status.get('detected', False):
            self._logger.debug("[AnafiTracker] Target not detected - waiting...")
            return
        
        # 단계별 처리
        if self._phase == TrackingPhase.CENTERING:
            self._execute_centering(status)
        elif self._phase == TrackingPhase.DISTANCE_ADJUST:
            self._execute_distance_adjust(status)
    
    # ==================== Tracking Logic ====================
    
    def _execute_centering(self, status: dict):
        """Phase 1: 좌우/상하 정렬"""
        offset_x_norm = status.get('offset_x_normalized', 0.0)
        offset_y_norm = status.get('offset_y_normalized', 0.0)
        
        # 이동량 계산
        dy = offset_x_norm * self._config.move_scale
        dz = offset_y_norm * self._config.move_scale
        
        # 최소 이동 임계값 적용
        if abs(dy) < self._config.min_move:
            dy = 0.0
        if abs(dz) < self._config.min_move:
            dz = 0.0
        
        # 최대 이동 제한
        dy = max(-self._config.max_move, min(self._config.max_move, dy))
        dz = max(-self._config.max_move, min(self._config.max_move, dz))
        
        if dy != 0.0 or dz != 0.0:
            self._logger.info(
                f"[AnafiTracker] Centering: dy={dy:.3f}m, dz={dz:.3f}m "
                f"(offset: x={offset_x_norm:.2f}, y={offset_y_norm:.2f})"
            )
            self._publish_moveby(dx=0.0, dy=dy, dz=dz)
        else:
            # 중앙 정렬 완료 → Phase 2로 전환
            self._phase = TrackingPhase.DISTANCE_ADJUST
            self._no_move_count = 0
            self._logger.warning("[AnafiTracker] ★ Centering COMPLETE! Moving to Phase 2: Distance Adjust ★")
    
    def _execute_distance_adjust(self, status: dict):
        """Phase 2: 앞뒤 거리 조절 (bbox 크기 기반)"""
        bbox_width = status.get('bbox_width', 0.0)
        bbox_height = status.get('bbox_height', 0.0)
        
        # 목표 범위 계산
        cfg = self._config
        width_min = cfg.target_bbox_width * (1.0 - cfg.bbox_tolerance)
        width_max = cfg.target_bbox_width * (1.0 + cfg.bbox_tolerance)
        height_min = cfg.target_bbox_height * (1.0 - cfg.bbox_tolerance)
        height_max = cfg.target_bbox_height * (1.0 + cfg.bbox_tolerance)
        
        width_ok = width_min <= bbox_width <= width_max
        height_ok = height_min <= bbox_height <= height_max
        
        self._logger.info(
            f"[AnafiTracker] Distance check: bbox={bbox_width:.0f}x{bbox_height:.0f} "
            f"(target: {cfg.target_bbox_width:.0f}x{cfg.target_bbox_height:.0f} ±{cfg.bbox_tolerance*100:.0f}%)"
        )
        
        if width_ok and height_ok:
            # 모든 조건 충족 - 추적 완료!
            self._complete_tracking()
            return
        
        # bbox 크기로 앞뒤 조절
        dx = 0.0
        if bbox_width < width_min or bbox_height < height_min:
            dx = cfg.dx_step  # 앞으로 (가까이)
            self._logger.info(f"[AnafiTracker] bbox too small → Forward: dx={dx:.3f}m")
        elif bbox_width > width_max or bbox_height > height_max:
            dx = -cfg.dx_step  # 뒤로 (멀리)
            self._logger.info(f"[AnafiTracker] bbox too large → Backward: dx={dx:.3f}m")
        
        if dx != 0.0:
            self._publish_moveby(dx=dx, dy=0.0, dz=0.0)
    
    def _complete_tracking(self):
        """추적 완료 처리"""
        self._phase = TrackingPhase.COMPLETED
        self._enabled = False
        
        # YOLO에 추적 비활성화 알림
        msg = Bool()
        msg.data = False
        self.pub_tracking_enable.publish(msg)
        
        # 타이머 중지
        self._stop_timer()
        
        self._logger.warning("[AnafiTracker] ★★★ TRACKING COMPLETE! ★★★")
        
        # 완료 콜백 호출
        if self._on_complete:
            self._on_complete()
    
    def _publish_moveby(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dyaw: float = 0.0):
        """MoveBy 명령 발행"""
        # 이동 전 위치 저장
        if self._current_position is not None:
            self._position_before_move = self._current_position
        
        msg = MoveByCommand()
        msg.dx = float(dx)
        msg.dy = float(dy)
        msg.dz = float(dz)
        msg.dyaw = float(dyaw)
        
        self._move_pending = True
        self.pub_moveby.publish(msg)
        
        self._logger.info(
            f"[AnafiTracker] MoveBy → dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}, dyaw={dyaw:.3f}"
        )

