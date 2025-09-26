#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def yaw_from_quat(w, x, y, z):
    # ENU, ZYX
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

def wrap_pi(a):
    while a > math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a


class AnafiVelFollower(Node):
    """
    Crazyflie의 '이동성'만 보고 Anafi가 따라가도록 하는 단순 추종 노드.
    - mode=velocity: CF 속도를 그대로(게인/포화만) 따라감
    - mode=gap_follow: 속도 + 간단한 거리 유지만 추가(1m 뒤 등)

    Sub:
      /cf/odom (nav_msgs/Odometry)      # pos, vel
      /cf/rpy  (geometry_msgs/Vector3Stamped)  # yaw(deg)
      /anafi/odom (nav_msgs/Odometry)
    Pub:
      /anafi/cmd_vel (geometry_msgs/TwistStamped)  # 바디 프레임 속도 + yaw_rate
    """

    def __init__(self):
        super().__init__('anafi_vel_follower')

        # -------- Parameters --------
        self.declare_parameter('mode', 'velocity')  # 'velocity' or 'gap_follow'

        self.declare_parameter('cf_odom_topic', '/cf/odom')
        self.declare_parameter('cf_rpy_topic',  '/cf/rpy')
        self.declare_parameter('anafi_odom_topic', '/anafi/odom')
        self.declare_parameter('anafi_cmd_topic',  '/anafi/cmd_vel')

        # velocity 모드 게인/포화
        self.declare_parameter('vel_gain_xy', 1.0)     # CF 속도에 곱할 이득
        self.declare_parameter('vel_gain_z',  1.0)
        self.declare_parameter('max_horiz_speed', 1.5)
        self.declare_parameter('max_vert_speed',  0.8)

        # gap_follow 전용: 간단한 거리 유지
        self.declare_parameter('follow_dist_m', 1.0)   # CF 뒤쪽 목표 간격
        self.declare_parameter('kp_gap', 0.6)          # 거리 보정 이득(맵 프레임)
        self.declare_parameter('vertical_offset_m', 0.0)
        self.declare_parameter('lateral_offset_m', 0.0)

        # yaw 정렬 옵션(두 모드 공통)
        self.declare_parameter('use_yaw_align', True)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)    # rad/s

        # 안전 고도
        self.declare_parameter('min_alt', 0.3)
        self.declare_parameter('max_alt', 6.0)

        # 명령 주기/타임아웃
        self.declare_parameter('cmd_rate_hz', 30.0)
        self.declare_parameter('target_timeout_s', 0.6)

        # 자동 이륙(선택)
        self.declare_parameter('auto_takeoff', False)
        self.declare_parameter('takeoff_srv', '/anafi/takeoff')

        g = lambda n: self.get_parameter(n).get_parameter_value()
        self.mode = g('mode').string_value

        self.cf_odom_topic = g('cf_odom_topic').string_value
        self.cf_rpy_topic  = g('cf_rpy_topic').string_value
        self.anafi_odom_topic = g('anafi_odom_topic').string_value
        self.anafi_cmd_topic  = g('anafi_cmd_topic').string_value

        self.vel_gain_xy = float(g('vel_gain_xy').double_value)
        self.vel_gain_z  = float(g('vel_gain_z').double_value)
        self.max_vxy = float(g('max_horiz_speed').double_value)
        self.max_vz  = float(g('max_vert_speed').double_value)

        self.follow_dist = float(g('follow_dist_m').double_value)
        self.kp_gap = float(g('kp_gap').double_value)
        self.vert_off = float(g('vertical_offset_m').double_value)
        self.lat_off  = float(g('lateral_offset_m').double_value)

        self.use_yaw_align = bool(g('use_yaw_align').bool_value)
        self.kp_yaw = float(g('kp_yaw').double_value)
        self.max_r  = float(g('max_yaw_rate').double_value)

        self.min_alt = float(g('min_alt').double_value)
        self.max_alt = float(g('max_alt').double_value)

        self.cmd_rate = float(g('cmd_rate_hz').double_value)
        self.tgt_timeout = float(g('target_timeout_s').double_value)

        self.auto_takeoff = bool(g('auto_takeoff').bool_value)
        self.takeoff_srv_name = g('takeoff_srv').string_value

        # -------- QoS --------
        sense_qos = QoSProfile(depth=20)
        sense_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sense_qos.history = HistoryPolicy.KEEP_LAST

        ctrl_qos = QoSProfile(depth=10)
        ctrl_qos.reliability = ReliabilityPolicy.RELIABLE
        ctrl_qos.history = HistoryPolicy.KEEP_LAST

        # -------- Subs/Pubs --------
        self.sub_cf_odom = self.create_subscription(Odometry, self.cf_odom_topic, self._on_cf_odom, sense_qos)
        self.sub_cf_rpy  = self.create_subscription(Vector3Stamped, self.cf_rpy_topic, self._on_cf_rpy, sense_qos)
        self.sub_anafi_odom = self.create_subscription(Odometry, self.anafi_odom_topic, self._on_anafi_odom, sense_qos)
        self.pub_cmd = self.create_publisher(TwistStamped, self.anafi_cmd_topic, ctrl_qos)

        # 서비스
        self.cli_takeoff = self.create_client(Trigger, self.takeoff_srv_name)

        # -------- States --------
        self.cf_pos: Optional[Tuple[float,float,float]] = None
        self.cf_vel: Optional[Tuple[float,float,float]] = None
        self.cf_yaw: Optional[float] = None
        self.cf_last_t: Optional[float] = None

        self.anafi_pos = [None, None, None]
        self.anafi_yaw: Optional[float] = None
        self.anafi_last_t: Optional[float] = None

        self.timer = self.create_timer(1.0/max(self.cmd_rate,1.0), self._control_step)
        self._tried_auto_takeoff = False

        self.get_logger().info(f'Started vel-follower in mode={self.mode}')

    # -------- Callbacks --------
    def _on_cf_odom(self, msg: Odometry):
        self.cf_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.cf_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.cf_last_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_cf_rpy(self, msg: Vector3Stamped):
        self.cf_yaw = math.radians(float(msg.vector.z))
        self.cf_last_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_anafi_odom(self, msg: Odometry):
        self.anafi_pos = [msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z]
        q = msg.pose.pose.orientation
        self.anafi_yaw = yaw_from_quat(q.w, q.x, q.y, q.z)
        self.anafi_last_t = self.get_clock().now().nanoseconds * 1e-9

    # -------- Helpers --------
    def _ready(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
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

    # -------- Control --------
    def _control_step(self):
        self._auto_takeoff()

        if not self._ready() or self.cf_vel is None:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            return

        ax, ay, az = self.anafi_pos
        yaw_a = self.anafi_yaw

        # 1) 기본: CF 속도를 지도 프레임에서 그대로 사용
        vx_map = self.vel_gain_xy * float(self.cf_vel[0])
        vy_map = self.vel_gain_xy * float(self.cf_vel[1])
        vz_map = self.vel_gain_z  * float(self.cf_vel[2])

        # 2) (선택) 간격 유지용 간단 오차 보정
        if self.mode.lower() == 'gap_follow' and (self.cf_pos is not None):
            if self.cf_yaw is not None:
                # CF 진행방향 기준 뒤쪽 follow_dist + 측면/수직 오프셋 적용
                fx, fy = math.cos(self.cf_yaw), math.sin(self.cf_yaw)
                lx, ly = -math.sin(self.cf_yaw), math.cos(self.cf_yaw)
                tx = self.cf_pos[0] - self.follow_dist * fx + self.lat_off * lx
                ty = self.cf_pos[1] - self.follow_dist * fy + self.lat_off * ly
            else:
                # yaw 없으면 간단히 CF 위치 자체를 목표로
                tx, ty = self.cf_pos[0], self.cf_pos[1]
            tz = clamp(self.cf_pos[2] + self.vert_off, self.min_alt, self.max_alt)

            ex, ey, ez = (tx - ax), (ty - ay), (tz - az)
            vx_map += self.kp_gap * ex
            vy_map += self.kp_gap * ey
            vz_map += self.kp_gap * ez
        else:
            # velocity 모드에서도 z는 CF(+오프셋) 쪽으로 약하게 수렴
            if self.cf_pos is not None:
                z_ref = clamp(self.cf_pos[2] + self.vert_off, self.min_alt, self.max_alt)
                vz_map += self.kp_gap * (z_ref - az)

        # 3) 지도 → 바디 변환
        c, s = math.cos(yaw_a), math.sin(yaw_a)
        vx_body =  c * vx_map + s * vy_map
        vy_body = -s * vx_map + c * vy_map
        vz_body = vz_map

        # 4) 포화
        vxy = math.hypot(vx_body, vy_body)
        if vxy > self.max_vxy:
            sc = self.max_vxy / max(vxy, 1e-6)
            vx_body *= sc; vy_body *= sc
        vz_body = clamp(vz_body, -self.max_vz, self.max_vz)

        # 5) Yaw 정렬(옵션)
        yaw_rate = 0.0
        if self.use_yaw_align and (self.cf_yaw is not None):
            yaw_err = wrap_pi(self.cf_yaw - yaw_a)
            yaw_rate = clamp(self.kp_yaw * yaw_err, -self.max_r, self.max_r)

        self._publish_cmd(vx_body, vy_body, vz_body, yaw_rate)

    def _publish_cmd(self, vx, vy, vz, r):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(r)
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = AnafiVelFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
