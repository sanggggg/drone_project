#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def yaw_from_quat(w, x, y, z):
    # ENU, ZYX
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def quat_from_yaw(yaw: float):
    # roll=pitch=0, yaw만 반영(ENU)
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w

def wrap_pi(a):
    while a > math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a


class AnafiPoseOffsetTrail(Node):
    """
    크레이지플라이 뒤를 '고정 거리 오프셋(기본 0.5 m)'로 따라가도록
    Anafi AI에 PoseStamped(위치+방향) 목표를 지속 전송하는 노드.

    아이디어:
      - CF의 yaw(또는 속도 방향) 기준으로 follow_dist_m 만큼 뒤쪽에 목표점(Target)을 생성
      - 필요시 좌/수직 오프셋 추가
      - Anafi에는 해당 목표점을 PoseStamped로 발행(지속 갱신)

    Sub:
      /cf/odom    (nav_msgs/Odometry)            # CF pos, vel
      /cf/rpy     (geometry_msgs/Vector3Stamped) # CF yaw(deg). 없으면 속도방향 사용
      /anafi/odom (nav_msgs/Odometry)            # 안전 경계/참조용

    Pub:
      /anafi/cmd_pose (geometry_msgs/PoseStamped)  # 기본값. 파라미터로 변경 가능
    """

    def __init__(self):
        super().__init__('anafi_pose_offset_trail')

        # ---------- Parameters ----------
        # 토픽
        self.declare_parameter('cf_odom_topic', '/cf/odom')
        self.declare_parameter('cf_rpy_topic',  '/cf/rpy')
        self.declare_parameter('anafi_odom_topic', '/anafi/odom')
        self.declare_parameter('anafi_pose_topic', '/anafi/cmd_pose')  # Anafi 위치 명령 토픽

        # 오프셋(뒤따르기)
        self.declare_parameter('follow_dist_m', 0.5)    # CF 뒤쪽 간격(기본 0.5 m)
        self.declare_parameter('lateral_offset_m', 0.0) # CF 좌(+)/우(-) 오프셋
        self.declare_parameter('vertical_offset_m', 0.0)# CF 대비 고도 오프셋

        # yaw 정렬 방식
        self.declare_parameter('yaw_mode', 'align_cf')  # 'align_cf' | 'keep_anafi' | 'face_target'
        self.declare_parameter('max_yaw_rate', 1.0)     # rad/s, face_target 보정 속도 제한용(부드럽게 회전)

        # 안전/경계
        self.declare_parameter('min_alt', 0.3)
        self.declare_parameter('max_alt', 6.0)

        # 명령 주기/타임아웃
        self.declare_parameter('cmd_rate_hz', 30.0)
        self.declare_parameter('target_timeout_s', 0.6)

        # 부드러운 목표 이동(저역통과)
        self.declare_parameter('pos_smoothing_tau_s', 0.15)  # 1차 LPF 시정수(작을수록 민첩, 0이면 끔)

        # 자동 이륙(선택)
        self.declare_parameter('auto_takeoff', False)
        self.declare_parameter('takeoff_srv', '/anafi/takeoff')

        # ---------- Load ----------
        g = lambda n: self.get_parameter(n).get_parameter_value()
        self.cf_odom_topic = g('cf_odom_topic').string_value
        self.cf_rpy_topic  = g('cf_rpy_topic').string_value
        self.anafi_odom_topic = g('anafi_odom_topic').string_value
        self.anafi_pose_topic  = g('anafi_pose_topic').string_value

        self.follow_dist = float(g('follow_dist_m').double_value)
        self.lat_off  = float(g('lateral_offset_m').double_value)
        self.vert_off = float(g('vertical_offset_m').double_value)

        self.yaw_mode = (g('yaw_mode').string_value or 'align_cf').lower()
        self.max_yaw_rate = float(g('max_yaw_rate').double_value)

        self.min_alt = float(g('min_alt').double_value)
        self.max_alt = float(g('max_alt').double_value)

        self.cmd_rate = float(g('cmd_rate_hz').double_value)
        self.tgt_timeout = float(g('target_timeout_s').double_value)

        self.tau = float(g('pos_smoothing_tau_s').double_value)  # 0이면 비활성

        self.auto_takeoff = bool(g('auto_takeoff').bool_value)
        self.takeoff_srv_name = g('takeoff_srv').string_value

        # ---------- QoS ----------
        sense_qos = QoSProfile(depth=30)
        sense_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sense_qos.history = HistoryPolicy.KEEP_LAST

        ctrl_qos = QoSProfile(depth=10)
        ctrl_qos.reliability = ReliabilityPolicy.RELIABLE
        ctrl_qos.history = HistoryPolicy.KEEP_LAST

        # ---------- Subs/Pubs ----------
        self.sub_cf_odom = self.create_subscription(Odometry, self.cf_odom_topic, self._on_cf_odom, sense_qos)
        self.sub_cf_rpy  = self.create_subscription(Vector3Stamped, self.cf_rpy_topic, self._on_cf_rpy, sense_qos)
        self.sub_anafi_odom = self.create_subscription(Odometry, self.anafi_odom_topic, self._on_anafi_odom, sense_qos)
        self.pub_pose = self.create_publisher(PoseStamped, self.anafi_pose_topic, ctrl_qos)

        # 서비스
        self.cli_takeoff = self.create_client(Trigger, self.takeoff_srv_name)

        # ---------- States ----------
        self.cf_pos: Optional[Tuple[float,float,float]] = None
        self.cf_vel: Optional[Tuple[float,float,float]] = None
        self.cf_yaw: Optional[float] = None
        self.cf_last_t: Optional[float] = None

        self.anafi_pos = [None, None, None]
        self.anafi_yaw: Optional[float] = None
        self.anafi_last_t: Optional[float] = None

        # yaw fallback(속도→방향)
        self._last_cf_yaw_fallback: Optional[float] = None

        # 스무딩용(목표 포즈 내부 상태)
        self._t_prev = None
        self._tx, self._ty, self._tz = None, None, None
        self._yaw_target = None

        # 타이머
        self.timer = self.create_timer(1.0/max(self.cmd_rate,1.0), self._step)
        self._tried_auto_takeoff = False

        self.get_logger().info(f'Started pose offset-trail: follow_dist={self.follow_dist:.2f} m, yaw_mode={self.yaw_mode}')

    # ---------- Callbacks ----------
    def _on_cf_odom(self, msg: Odometry):
        self.cf_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.cf_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.cf_last_t = self._now()

        vxy = math.hypot(self.cf_vel[0], self.cf_vel[1])
        if vxy > 0.05:
            self._last_cf_yaw_fallback = math.atan2(self.cf_vel[1], self.cf_vel[0])

    def _on_cf_rpy(self, msg: Vector3Stamped):
        self.cf_yaw = math.radians(float(msg.vector.z))
        self.cf_last_t = self._now()

    def _on_anafi_odom(self, msg: Odometry):
        self.anafi_pos = [msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.anafi_yaw = yaw_from_quat(q.w, q.x, q.y, q.z)
        self.anafi_last_t = self._now()

    # ---------- Helpers ----------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ready(self) -> bool:
        now = self._now()
        if (self.cf_last_t is None) or (now - self.cf_last_t > self.tgt_timeout):
            return False
        if any(v is None for v in self.anafi_pos) or self.anafi_yaw is None:
            return False
        return True

    def _auto_takeoff(self):
        if not self.auto_takeoff or self._tried_auto_takeoff:
            return
        if self.cli_takeoff.service_is_ready():
            self.get_logger().info('Calling /anafi/takeoff ...')
            self.cli_takeoff.call_async(Trigger.Request())
            self._tried_auto_takeoff = True

    def _pick_cf_yaw(self) -> Optional[float]:
        if self.cf_yaw is not None:
            return self.cf_yaw
        if self._last_cf_yaw_fallback is not None:
            return self._last_cf_yaw_fallback
        return None

    # ---------- Control ----------
    def _step(self):
        self._auto_takeoff()

        now = self._now()
        if not self._ready() or self.cf_pos is None:
            # 대상 없으면 마지막 목표 유지(혹은 안전 고도/현재 위치 유지)
            if self.anafi_pos[0] is not None:
                self._publish_pose(self.anafi_pos[0], self.anafi_pos[1],
                                   clamp(self.anafi_pos[2] if self.anafi_pos[2] is not None else 0.5,
                                         self.min_alt, self.max_alt),
                                   self.anafi_yaw if self.anafi_yaw is not None else 0.0)
            self._t_prev = now
            return

        ax, ay, az = self.anafi_pos
        a_yaw = self.anafi_yaw

        # 1) CF 진행방향 얻기
        cf_yaw = self._pick_cf_yaw()
        if cf_yaw is None:
            # 방향을 모르면 현재 Anafi yaw 유지 + 위치만 현재 위치 유지
            self._publish_pose(ax, ay, clamp(az if az is not None else self.min_alt, self.min_alt, self.max_alt), a_yaw or 0.0)
            self._t_prev = now
            return

        # 2) 목표점 생성(지도 프레임): CF 뒤쪽 follow_dist + 좌/수직 오프셋
        fx, fy = math.cos(cf_yaw), math.sin(cf_yaw)        # 진행방향
        lx, ly = -math.sin(cf_yaw), math.cos(cf_yaw)       # 좌측
        tx_raw = self.cf_pos[0] - self.follow_dist * fx + self.lat_off * lx
        ty_raw = self.cf_pos[1] - self.follow_dist * fy + self.lat_off * ly
        tz_raw = clamp(self.cf_pos[2] + self.vert_off, self.min_alt, self.max_alt)

        # 3) 목표 yaw 결정
        if self.yaw_mode == 'align_cf':
            yaw_target = cf_yaw
        elif self.yaw_mode == 'face_target':
            # 목표점 쪽을 바라보도록 부드럽게 회전
            dyaw = math.atan2((ty_raw - ay), (tx_raw - ax))
            # 제한된 회전 속도로 추종(여기서는 간단히 선형 보정)
            err = wrap_pi(dyaw - a_yaw)
            yaw_target = a_yaw + clamp(err, -self.max_yaw_rate / max(self.cmd_rate,1.0),
                                             self.max_yaw_rate / max(self.cmd_rate,1.0))
        else:  # 'keep_anafi'
            yaw_target = a_yaw

        # 4) 목표 포즈 스무딩(선택)
        if self.tau > 1e-6 and self._t_prev is not None and self._tx is not None:
            dt = max(1e-3, now - self._t_prev)
            alpha = clamp(dt / max(self.tau, 1e-6), 0.0, 1.0)  # 1차 LPF 계수
            tx = (1.0 - alpha) * self._tx + alpha * tx_raw
            ty = (1.0 - alpha) * self._ty + alpha * ty_raw
            tz = (1.0 - alpha) * self._tz + alpha * tz_raw
            # yaw도 부드럽게(옵션)
            dy = wrap_pi(yaw_target - (self._yaw_target if self._yaw_target is not None else yaw_target))
            yaw_s = (self._yaw_target if self._yaw_target is not None else yaw_target) + alpha * dy
        else:
            tx, ty, tz = tx_raw, ty_raw, tz_raw
            yaw_s = yaw_target

        # 5) 목표 포즈 발행
        self._publish_pose(tx, ty, tz, yaw_s)

        # 6) 내부 상태 갱신
        self._tx, self._ty, self._tz = tx, ty, tz
        self._yaw_target = yaw_s
        self._t_prev = now

    def _publish_pose(self, x: float, y: float, z: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # 필요 시 변경
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)


def main():
    rclpy.init()
    node = AnafiPoseOffsetTrail()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
