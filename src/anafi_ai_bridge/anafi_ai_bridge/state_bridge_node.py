import math
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Twist

import olympe
from olympe.messages.ardrone3.PilotingState import AttitudeChanged, SpeedChanged, PositionChanged
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.ardrone3.Piloting import PCMD


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def clamp(v, lo, hi):  # 편의
    return max(lo, min(hi, v))


class AnafiStateBridge(Node):
    """
    퍼블리시:
      - /anafi/pose (geometry_msgs/PoseStamped)
      - /anafi/navsatfix (sensor_msgs/NavSatFix)
      - /anafi/twist (geometry_msgs/TwistStamped)
      - /anafi/battery (sensor_msgs/BatteryState)
    구독:
      - /anafi/cmd_vel (geometry_msgs/Twist)  -> Olympe PCMD 변환 전송
    """

    def __init__(self):
        super().__init__("anafi_state_bridge")

        # ---- Parameters
        self.declare_parameter("ip", "192.168.53.1")
        self.declare_parameter("base_frame", "anafi/base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("angles_in_degrees", True)
        self.declare_parameter("fill_pose_altitude", True)

        # PCMD 전송 관련 파라미터
        self.declare_parameter("enable_control", True)
        self.declare_parameter("pcmd_rate_hz", 25.0)
        self.declare_parameter("deadman_timeout", 0.5)     # [s] 최근 cmd_vel 없으면 정지
        # cmd_vel → PCMD 스케일 (m/s, rad/s → [-100..100]% 맵핑 기준)
        self.declare_parameter("max_vx", 2.0)              # forward/back
        self.declare_parameter("max_vy", 2.0)              # left/right
        self.declare_parameter("max_vz", 1.0)              # up/down
        self.declare_parameter("max_yaw_rate", 1.0)        # rad/s

        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.angles_in_degrees = self.get_parameter("angles_in_degrees").get_parameter_value().bool_value
        self.fill_pose_altitude = self.get_parameter("fill_pose_altitude").get_parameter_value().bool_value

        self.enable_control = self.get_parameter("enable_control").get_parameter_value().bool_value
        self.pcmd_rate_hz = float(self.get_parameter("pcmd_rate_hz").get_parameter_value().double_value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").get_parameter_value().double_value)
        self.max_vx = float(self.get_parameter("max_vx").get_parameter_value().double_value)
        self.max_vy = float(self.get_parameter("max_vy").get_parameter_value().double_value)
        self.max_vz = float(self.get_parameter("max_vz").get_parameter_value().double_value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").get_parameter_value().double_value)

        # QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_pose = self.create_publisher(PoseStamped, "/anafi/pose", qos)
        self.pub_fix = self.create_publisher(NavSatFix, "/anafi/navsatfix", qos)
        self.pub_twist = self.create_publisher(TwistStamped, "/anafi/twist", qos)
        self.pub_batt = self.create_publisher(BatteryState, "/anafi/battery", qos)

        # cmd_vel 구독
        self._last_cmd_time = 0.0
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)  # roll, pitch, yaw, gaz [%]
        if self.enable_control:
            self.create_subscription(Twist, "/anafi/cmd_vel", self._cmd_cb, 10)

        # Olympe 연결
        self.drone = olympe.Drone(self.ip)
        if not self.drone.connect():
            self.get_logger().error(f"Olympe connect failed: {self.ip}")
        else:
            self.get_logger().info(f"Connected to {self.ip}")

        # 퍼블리시/PCMD 루프
        self._stop = False
        self.thread_state = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread_state.start()
        if self.enable_control:
            self.thread_pcmd = threading.Thread(target=self._pcmd_loop, daemon=True)
            self.thread_pcmd.start()

        # 종료 핸들러
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    # ---- cmd_vel → PCMD 변환
    def _cmd_cb(self, msg: Twist):
        # cmd_vel (ENU): x(forward +), y(left +), z(up +), yaw CCW +
        # PCMD: roll(left/right), pitch(forward/back), yaw, gaz(up/down)  각 [-100..100]
        pitch = clamp((msg.linear.x / self.max_vx) * 100.0, -100.0, 100.0)
        roll  = clamp((msg.linear.y / self.max_vy) * 100.0, -100.0, 100.0)
        gaz   = clamp((msg.linear.z / self.max_vz) * 100.0, -100.0, 100.0)
        yaw   = clamp((msg.angular.z / self.max_yaw_rate) * 100.0, -100.0, 100.0)
        self._last_cmd = (roll, pitch, yaw, gaz)
        self._last_cmd_time = time.time()

    def _pcmd_loop(self):
        period = 1.0 / max(self.pcmd_rate_hz, 1.0)
        while rclpy.ok() and not self._stop:
            now = time.time()
            roll, pitch, yaw, gaz = self._last_cmd
            active = (now - self._last_cmd_time) <= self.deadman_timeout
            flag = 1 if (active and (abs(roll) > 1e-3 or abs(pitch) > 1e-3)) else 0
            if not active:
                roll = pitch = yaw = gaz = 0.0
            try:
                self.drone(PCMD(
                    flag=flag,
                    roll=int(roll), pitch=int(pitch),
                    yaw=int(yaw), gaz=int(gaz),
                    timestampAndSeqNum=0
                ))
            except Exception as e:
                self.get_logger().warn(f"PCMD send error: {e}")
            time.sleep(period)

    # ---- 상태 퍼블리시 루프
    def _publish_loop(self):
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        while rclpy.ok() and not self._stop:
            try:
                att = self.drone.get_state(AttitudeChanged())
                spd = self.drone.get_state(SpeedChanged())
                pos = self.drone.get_state(PositionChanged())
                bat = self.drone.get_state(BatteryStateChanged())
            except Exception as e:
                self.get_logger().warn(f"Olympe get_state error: {e}")
                time.sleep(period)
                continue

            now = self.get_clock().now().to_msg()

            # Pose (orientation + optional altitude)
            ps = PoseStamped()
            ps.header = Header(stamp=now, frame_id=self.map_frame)
            roll = float(att.get("roll", 0.0))
            pitch = float(att.get("pitch", 0.0))
            yaw = float(att.get("yaw", 0.0))
            if self.angles_in_degrees:
                roll *= math.pi/180.0; pitch *= math.pi/180.0; yaw *= math.pi/180.0
            ps.pose.orientation = euler_to_quaternion(roll, pitch, yaw)
            if self.fill_pose_altitude and pos.get("altitude") is not None:
                ps.pose.position.z = float(pos["altitude"])
            self.pub_pose.publish(ps)

            # GPS
            fix = NavSatFix()
            fix.header = Header(stamp=now, frame_id=self.map_frame)
            lat = pos.get("latitude", None); lon = pos.get("longitude", None); alt = pos.get("altitude", None)
            if lat is None or lon is None:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
                fix.latitude = float("nan"); fix.longitude = float("nan"); fix.altitude = float("nan")
            else:
                fix.status.status = NavSatStatus.STATUS_FIX
                fix.latitude = float(lat); fix.longitude = float(lon)
                fix.altitude = float(alt) if alt is not None else float("nan")
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub_fix.publish(fix)

            # 속도 (ARSDK: X front, Y right, Z down) → ENU(base_link): X fwd, Y left, Z up
            ts = TwistStamped()
            ts.header = Header(stamp=now, frame_id=self.base_frame)
            vx = float(spd.get("speedX", 0.0)); vy = float(spd.get("speedY", 0.0)); vz = float(spd.get("speedZ", 0.0))
            ts.twist.linear.x = vx
            ts.twist.linear.y = -vy
            ts.twist.linear.z = -vz
            self.pub_twist.publish(ts)

            # 배터리
            bs = BatteryState()
            bs.header = Header(stamp=now, frame_id=self.base_frame)
            percent = bat.get("percent", None)
            bs.percentage = float(percent)/100.0 if percent is not None else float("nan")
            self.pub_batt.publish(bs)

            time.sleep(period)

        try:
            self.drone.disconnect()
        except Exception:
            pass

    def _on_signal(self, *args):
        self._stop = True

    def destroy_node(self):
        self._stop = True
        try:
            self.thread_state.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, "thread_pcmd"): self.thread_pcmd.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.drone.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AnafiStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
